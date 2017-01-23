#include "dataload_fb.h"
#include <QDataStream>
#include <QByteArray>
#include <QFile>
#include <QMessageBox>
#include "selectxaxisdialog.h"
#include <QDebug>
#include <iostream>
#include <functional>
#include "flatbuffers/idl.h"
#include "lz4.h"

enum BaseType {
    None = 0,
    Bool = 1,
    Byte = 2,
    UByte = 3,
    Short = 4,
    UShort = 5,
    Int = 6,
    UInt = 7,
    Long = 8,
    ULong = 9,
    Float = 10,
    Double = 11,
    MIN = None,
    MAX = Double
};

inline const char **EnumNamesBaseType() {
    static const char *names[] = { "none", "bool",      "byte", "ubyte",
                                   "short", "ushort",   "int",  "uint",
                                   "long", "ulong",     "float", "double",
                                   nullptr };
    return names;
}

inline BaseType BaseTypeFromString( const char* type_name)
{
    const char **names = EnumNamesBaseType();

    for (int i=1; i<= BaseType::MAX; i++)
    {
        if( strcmp( names[i], type_name) == 0)
        {
            return static_cast<BaseType>( i );
        }
    }
    return BaseType::None;
}

inline int SizeOf(BaseType e) {
    static int sizes[] = { 0, 1, 1, 1,
                           2, 2, 4, 4,
                           8,8, 4, 8 };
    return sizes[static_cast<int>(e)];
}


DataLoadFlatbuffer::DataLoadFlatbuffer()
{
    _extensions.push_back( "pms");
}

const std::vector<const char*> &DataLoadFlatbuffer::compatibleFileExtensions() const
{
    return _extensions;
}


QString extractSchemaFromHeader(QDataStream* stream )
{
    QString header, line;

    int can_read = 1;

    while( can_read )
    {
        line.clear();
        char c = 0;
        do{
            can_read = stream->readRawData( &c, 1 );
            if( can_read == 0 ){
                break;
            }
            line.append( c );
        }while ( c !='\n' );

        if( line.contains( "COMPRESSED_DATA_STARTS_NEXT" ) )
        {
            break;
        }
        else{
            if( line.size() !=0  )
                header.append( line );
        }
    }

    return header;
}

typedef struct{
    std::vector<uint64_t> offsets;
    std::vector<BaseType> types;
    QStringList  names;
    std::vector< std::function<double(void*) > > readFunction;

} Schema;

Schema parseSchemaAndGetOffsets(QString schema_text)
{
    Schema schema;
    schema.offsets.push_back( 0 );


    QStringList lines = schema_text.split(QRegExp("\n|\r\n|\r"),QString::SkipEmptyParts);

    for (int i=0; i< lines.size(); i++)
    {

        QString line = lines.at(i);
        QStringList items = line.split( QRegExp("[ :;]"),QString::SkipEmptyParts);

        schema.names.push_back( items.at(0) );

        QString type_name = items.at(1).toLower();

        if( type_name.compare("float") == 0) {
            schema.readFunction.push_back(
                        [](void* ptr) { return static_cast<double>( flatbuffers::ReadScalar<float>(ptr) ); } );
        }
        else if( type_name.compare("double") == 0) {
            schema.readFunction.push_back(
                        [](void* ptr) { return static_cast<double>( flatbuffers::ReadScalar<double>(ptr) ); } );
        }
        else if( type_name.compare("bool") == 0) {
            schema.readFunction.push_back(
                        [](void* ptr) { return static_cast<double>( flatbuffers::ReadScalar<bool>(ptr) ); } );
        }
        else if( type_name.compare("byte") == 0) {
            schema.readFunction.push_back(
                        [](void* ptr) { return static_cast<double>( flatbuffers::ReadScalar<int8_t>(ptr) ); } );
        }
        else if( type_name.compare("ubyte") == 0) {
            schema.readFunction.push_back(
                        [](void* ptr) { return static_cast<double>( flatbuffers::ReadScalar<uint8_t>(ptr) ); } );
        }
        else if( type_name.compare("short") == 0) {
            schema.readFunction.push_back(
                        [](void* ptr) { return static_cast<double>( flatbuffers::ReadScalar<int16_t>(ptr) ); } );
        }
        else if( type_name.compare("ushort") == 0) {
            schema.readFunction.push_back(
                        [](void* ptr) { return static_cast<double>( flatbuffers::ReadScalar<uint16_t>(ptr) ); } );
        }
        else if( type_name.compare("int") == 0) {
            schema.readFunction.push_back(
                        [](void* ptr) { return static_cast<double>( flatbuffers::ReadScalar<int32_t>(ptr) ); } );
        }
        else if( type_name.compare("uint") == 0) {
            schema.readFunction.push_back(
                        [](void* ptr) { return static_cast<double>( flatbuffers::ReadScalar<uint32_t>(ptr) ); } );
        }
        else if( type_name.compare("long") == 0) {
            schema.readFunction.push_back(
                        [](void* ptr) { return static_cast<double>( flatbuffers::ReadScalar<int64_t>(ptr) ); } );
        }
        else if( type_name.compare("ulong") == 0) {
            schema.readFunction.push_back(
                        [](void* ptr) { return static_cast<double>( flatbuffers::ReadScalar<uint64_t>(ptr) ); } );
        }
        else{
            throw std::runtime_error("type not recognized");
        }

        BaseType type = BaseTypeFromString( type_name.toStdString().c_str()) ;
        schema.types.push_back( type );
        schema.offsets.push_back(  schema.offsets.back() + SizeOf(type) );

    }
    return schema;
}


bool decompressBlock( QDataStream* source, LZ4_streamDecode_t* lz4_stream, std::vector<char>* output, size_t* parsed_size )
{
    if( source->atEnd())
    {
        return false;
    }

    const size_t BUFFER_SIZE = 1024*512 ;
    output->resize( BUFFER_SIZE );

    char comp_buffer[LZ4_COMPRESSBOUND( BUFFER_SIZE )];
    size_t block_size = 0;

    char c[8];
    source->readRawData( c, 8);

    for (int i=0; i<8; i++ )
    {
        block_size += ((uint8_t)c[i]) << (8*i);
    }

    if( block_size <= 0 || block_size > BUFFER_SIZE) {
        return false;
    }

    const size_t readCount =  source->readRawData( comp_buffer, (size_t) block_size);

    *parsed_size = 8+ readCount;

    if(readCount != (size_t) block_size) {
        return false;
    }

    const size_t decBytes = LZ4_decompress_safe_continue(
                lz4_stream,
                comp_buffer,
                output->data(),
                block_size,
                BUFFER_SIZE );

    if(decBytes <= 0) {
        return false;
    }
    output->resize(decBytes);

    qDebug() << block_size << " to " << decBytes;

    return true;
}


char* getSingleFlatbuffer(char* source, std::vector<uint8_t>* destination)
{
    uint8_t c;
    uint32_t chunk_size = 0;
    for (int i=0; i< sizeof(uint32_t); i++ )
    {
        c = (uint8_t)(*source);
        source++;
        chunk_size += c << (8*i);
    }

    if( destination) {

        destination->resize( chunk_size );
        for (uint32_t i=0; i< chunk_size; i++ )
        {
            (*destination)[i] = (uint8_t)(*source);
            source++;
        }
    }
    else{
        source += chunk_size ;
    }

    return source;
}


PlotDataMap DataLoadFlatbuffer::readDataFromFile(QFile *file,
                                                 std::function<void(int)> updateCompletion,
                                                 std::function<bool()> checkInterruption,
                                                 int time_index)
{
    float file_size = file->size();
    float parsed_size = 0;

    PlotDataMap plot_data;

    // get the schema from the header of the file
    QDataStream file_stream(file);

    file_stream.device()->setTextModeEnabled( false );

    //--------------------------------------
    QString schema_text = extractSchemaFromHeader( &file_stream );

    // parse the schema
    Schema schema = parseSchemaAndGetOffsets( schema_text );


    //--------------------------------------
    // choose the time axis
    if( time_index == TIME_INDEX_NOT_DEFINED)
    {
        QStringList field_names;
        field_names.push_back( "INDEX (auto-generated)" );
        field_names.append( schema.names);

        SelectXAxisDialog* dialog = new SelectXAxisDialog( &field_names );
        dialog->exec();
        time_index = dialog->getSelectedRowNumber().at(0) -1; // vector is supposed to have only one element
    }

    // allocate vectors

    for (int i=0; i< schema.names.size(); i++)
    {
        data_vectors.push_back( SharedVector(new boost::circular_buffer<double>()) );

        PlotDataPtr plot( new PlotData );
        std::string name = schema.names.at(i).toStdString();
        plot->setName( name );
        plot_data.insert( std::make_pair( name, plot ) );
    }

    //--------------------------------------

    size_t msg_count = 0;

    SharedVector time_vector (new boost::circular_buffer<double>() );

    bool interrupted = false;

    std::vector<char> decompressed_buffer;

    LZ4_streamDecode_t lz4_stream;
    LZ4_setStreamDecode( &lz4_stream, NULL, 0);

    size_t block_size;

    while( decompressBlock( &file_stream, &lz4_stream, &decompressed_buffer , &block_size)
           && !interrupted )
    {
        parsed_size += block_size;

        updateCompletion( (100.0*parsed_size)/file_size );
        interrupted = checkInterruption();

        size_t buffer_size   = decompressed_buffer.size();
        size_t message_size = schema.offsets.back();
        int num_messages = buffer_size/message_size;

        for (int m=0; m < num_messages; m++)
        {
            char *msg_ptr = & decompressed_buffer[ m*message_size ];

            if( time_index < 0) {
                time_vector->push_back( msg_count++ );
            }

            for (uint32_t f=0; f< schema.readFunction.size(); f++)
            {
                size_t offset = schema.offsets[f];
                double value = schema.readFunction[f]( msg_ptr + offset  );
                data_vectors[f]->push_back( value );
            }
        }
    }


    if(interrupted)
    {
        plot_data.erase( plot_data.begin(), plot_data.end() );
    }
    else{

        for( unsigned i=0; i < schema.names.size(); i++)
        {
            QString name = schema.names[i];
            plot_data[ name.toStdString() ]->addData( time_vector, data_vectors[i]);
        }
    }

    return plot_data;
}



DataLoadFlatbuffer::~DataLoadFlatbuffer()
{

}
