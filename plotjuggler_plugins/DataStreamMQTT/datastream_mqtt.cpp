#include "datastream_mqtt.h"
#include "ui_datastream_mqtt.h"
#include <QMessageBox>
#include <QSettings>
#include <QDebug>
#include <QUuid>
#include <QIntValidator>
#include <QMessageBox>

#ifdef WIN32
#include <windows.h>
#include <strsafe.h>
#endif

void connect_callback(struct mosquitto *mosq, void *context, int result,  int, const mosquitto_property *)
{
  DataStreamMQTT* _this = static_cast<DataStreamMQTT*>(context);
  auto& config = _this->_config;

  if( !result)
  {
    for(const auto& topic: config.topics)
    {
      mosquitto_subscribe(mosq, nullptr, topic.c_str(), config.qos);
    }
  }
  else
  {
     QMessageBox::warning(nullptr, "MQTT Client",
                           QString("Connection error: %1").arg(mosquitto_reason_string(result)),
                           QMessageBox::Ok);
  }
}

void disconnect_callback(struct mosquitto *mosq, void *context, int result)
{
  DataStreamMQTT* _this = static_cast<DataStreamMQTT*>(context);
  qDebug() << "disconnect callback, rc = " << result;
}

void message_callback(struct mosquitto *mosq, void *context, const struct mosquitto_message *message, const mosquitto_property *)
{
  DataStreamMQTT* _this = static_cast<DataStreamMQTT*>(context);

  std::unique_lock<std::mutex> lk(_this->mutex());

  auto it = _this->_parsers.find(message->topic);
  if( it == _this->_parsers.end() )
  {
    auto& parser_factory = _this->availableParsers()->at( _this->_protocol );
    auto parser = parser_factory->createInstance({}, _this->dataMap());
    it = _this->_parsers.insert( {message->topic, parser} ).first;
  }
  auto& parser = it->second;

  bool result = false;
  try {
    MessageRef msg( static_cast<uint8_t*>(message->payload), message->payloadlen);

    using namespace std::chrono;
    auto ts = high_resolution_clock::now().time_since_epoch();
    double timestamp = 1e-6* double( duration_cast<microseconds>(ts).count() );

    result = parser->parseMessage(msg, timestamp);

  } catch (std::exception& ) {
  }

  emit _this->dataReceived();
  if( !result )
  {
    _this->_failed_parsing++;
    emit _this->notificationsChanged(_this->_failed_parsing);
  }
}

/*
 *  mid -         the message id of the subscribe message.
 *  qos_count -   the number of granted subscriptions (size of granted_qos).
 *  granted_qos - an array of integers indicating the granted QoS for each of
 *                the subscriptions.
 */
void subscribe_callback(struct mosquitto *mosq, void *context, int mid, int qos_count, const int *granted_qos)
{
  DataStreamMQTT* _this = static_cast<DataStreamMQTT*>(context);
  _this->_subscribed = true;
}

void unsubscribe_callback(struct mosquitto *mosq, void *context, int result)
{
  DataStreamMQTT* _this = static_cast<DataStreamMQTT*>(context);
  qDebug() << QString("Subscription Failure. Code %1").arg(result);
  _this->_finished = true;
}

class MQTT_Dialog: public QDialog
{
public:
  MQTT_Dialog():
    QDialog(nullptr),
    ui(new Ui::DataStreamMQTT)
  {
    ui->setupUi(this);
    ui->lineEditPort->setValidator( new QIntValidator );

    static QString uuid =  QString("Plotjuggler-") + QString::number(rand());
    ui->lineEditClientID->setText(uuid);

    QSettings settings;
    restoreGeometry(settings.value("MosquittoMQTT::geometry").toByteArray());

    QString host = settings.value("MosquittoMQTT::host").toString();
    ui->lineEditHost->setText( host );

    int port = settings.value("MosquittoMQTT::port", 1883).toInt();
    ui->lineEditPort->setText( QString::number(port) );

    int qos = settings.value("MosquittoMQTT::qos", 0).toInt();
    ui->comboBoxQoS->setCurrentIndex(qos);

    int protocol_mqtt = settings.value("MosquittoMQTT::protocol_version").toInt();
    ui->comboBoxVersion->setCurrentIndex(protocol_mqtt);

    QString topic_filter = settings.value("MosquittoMQTT::filter").toString();
    ui->lineEditTopicFilter->setText(topic_filter );

    QString username = settings.value("MosquittoMQTT::username", "").toString();
    ui->lineEditUsername->setText(username);

    QString password = settings.value("MosquittoMQTT::password", "").toString();
    ui->lineEditPassword->setText(password);
  }

  void saveParameters(MosquittoConfig* config)
  {
    config->id = ui->lineEditClientID->text().toStdString();
    config->host = ui->lineEditHost->text().toStdString();
    config->port = ui->lineEditPort->text().toInt();
    config->topics.clear();
    config->topics.push_back(ui->lineEditTopicFilter->text().toStdString());
    config->username = ui->lineEditUsername->text().toStdString();
    config->password = ui->lineEditPassword->text().toStdString();
    config->qos = ui->comboBoxQoS->currentIndex();
    config->protocol_version =
        MQTT_PROTOCOL_V31 + ui->comboBoxVersion->currentIndex();

    config->keepalive = 60; // TODO
    config->bind_address = ""; //TODO
    config->will_retain = false; //TODO
    config->max_inflight = 20; //TODO
    config->clean_session = true;  //TODO
    config->eol = true;  //TODO
  }

  ~MQTT_Dialog()
  {
    while( ui->layoutOptions->count() > 0)
    {
      auto item = ui->layoutOptions->takeAt(0);
      item->widget()->setParent(nullptr);
    }

    QSettings settings;
    settings.setValue("MosquittoMQTT::geometry", this->saveGeometry());

    // save back to service
    settings.setValue("MosquittoMQTT::host", ui->lineEditHost->text());
    settings.setValue("MosquittoMQTT::port", ui->lineEditPort->text().toInt());
    settings.setValue("MosquittoMQTT::filter", ui->lineEditTopicFilter->text());
    settings.setValue("MosquittoMQTT::username", ui->lineEditPassword->text());
    settings.setValue("MosquittoMQTT::password", ui->lineEditUsername->text());
    settings.setValue("MosquittoMQTT::protocol_version", ui->comboBoxVersion->currentIndex());
    settings.setValue("MosquittoMQTT::qos", ui->comboBoxQoS->currentIndex());
    settings.setValue("MosquittoMQTT::serialization_protocol", ui->comboBoxProtocol->currentText());

    delete ui;
  }

  Ui::DataStreamMQTT* ui;
};

//---------------------------------------------
int SetClientOptions(struct mosquitto *mosq, struct MosquittoConfig *cfg)
{
  mosquitto_int_option(mosq, MOSQ_OPT_PROTOCOL_VERSION, cfg->protocol_version);

  if(!cfg->will_topic.empty())
  {
    if( mosquitto_will_set_v5(
            mosq,
            cfg->will_topic.c_str(),
            cfg->will_payload.size(),
            cfg->will_payload.c_str(),
            cfg->will_qos,
            cfg->will_retain, cfg->will_props))
    {
      return 1;
    }
  }

  cfg->will_props = nullptr;

  if(( !cfg->username.empty() || !cfg->password.empty()) )
  {
    if(mosquitto_username_pw_set(mosq, cfg->username.c_str(), cfg->password.c_str()))
    {
      return 1;
    }
  }

  mosquitto_max_inflight_messages_set(mosq, cfg->max_inflight);

  return 0;
}


DataStreamMQTT::DataStreamMQTT():
  _running(false)
{
  mosquitto_lib_init();

  _notification_action = new QAction(this);

  connect(_notification_action, &QAction::triggered, this, [this]() {
    QMessageBox::warning(nullptr, "MQTT error",
                         QString("Failed to parse %1 messages").arg(_failed_parsing),
                         QMessageBox::Ok);

    if (_failed_parsing > 0)
    {
      _failed_parsing = 0;
      emit notificationsChanged(_failed_parsing);
    }
  });
}

DataStreamMQTT::~DataStreamMQTT()
{
  shutdown();
  mosquitto_lib_cleanup();
}

bool DataStreamMQTT::start(QStringList *)
{
  if (_running)
  {
    return _running;
  }

  //cleanup notifications
  _failed_parsing = 0;
  emit notificationsChanged(0);

  if( !availableParsers() )
  {
    QMessageBox::warning(nullptr,tr("MQTT Client"), tr("No available MessageParsers"),  QMessageBox::Ok);
    _running = false;
    return false;
  }

  MQTT_Dialog dialog;

  for( const auto& it: *availableParsers())
  {
    dialog.ui->comboBoxProtocol->addItem( it.first );

    if(auto widget = it.second->optionsWidget() )
    {
      widget->setVisible(false);
      dialog.ui->layoutOptions->addWidget( widget );
    }
  }

  std::shared_ptr<MessageParserCreator> parser_creator;

  connect(dialog.ui->comboBoxProtocol,
          qOverload<const QString &>(&QComboBox::currentIndexChanged),
          this, [&](const QString & selected_protocol) {
            if (parser_creator)
            {
              if( auto prev_widget = parser_creator->optionsWidget())
              {
                prev_widget->setVisible(false);
              }
            }
            parser_creator = availableParsers()->at(selected_protocol);

            if (auto widget = parser_creator->optionsWidget())
            {
              widget->setVisible(true);
            }
          });

  QSettings settings;
  _protocol = settings.value("MosquittoMQTT::serialization_protocol", "JSON").toString();

  dialog.ui->comboBoxProtocol->setCurrentText(_protocol);

  if( dialog.exec() == QDialog::Rejected )
  {
    return false;
  }

  dialog.saveParameters(&_config);
  _protocol = dialog.ui->comboBoxProtocol->currentText();

  _subscribed = false;
  _finished = false;
  _running = false;

  for(const auto& filter: _config.topics)
  {
    if(mosquitto_sub_topic_check(filter.c_str()) == MOSQ_ERR_INVAL)
    {
      QMessageBox::warning(nullptr,tr("MQTT Client"),
                           tr("Error: Invalid subscription topic '%1', are all '+' and '#' wildcards correct?")
                               .arg(QString::fromStdString(filter)),
                           QMessageBox::Ok);
      return false;
    }
  }

//  _protocol_issue = false;
  if(_mosq)
  {
    mosquitto_destroy(_mosq);
  }
  _mosq = mosquitto_new(_config.id.c_str(), true, this);

  if(!_mosq)
  {
    QMessageBox::warning(nullptr,tr("MQTT Client"),
                         tr("Problem creating the Mosquitto client"),  QMessageBox::Ok);
    return false;
  }

  if(SetClientOptions(_mosq, &_config))
  {
    QMessageBox::warning(nullptr,tr("MQTT Client"),
                         tr("Problem seeting the Mosquitto options"),  QMessageBox::Ok);
    return false;
  }

  mosquitto_connect_v5_callback_set(_mosq, connect_callback);
  mosquitto_disconnect_callback_set(_mosq, disconnect_callback);
  mosquitto_message_v5_callback_set(_mosq, message_callback);

  mosquitto_subscribe_callback_set(_mosq, subscribe_callback);
  mosquitto_unsubscribe_callback_set(_mosq, unsubscribe_callback);

  const mosquitto_property *properties = nullptr; // todo

  int rc = mosquitto_connect_bind_v5(_mosq,
                                     _config.host.c_str(),
                                     _config.port,
                                     _config.keepalive,
                                     nullptr,
                                     properties); // TODO bind
  if(rc>0)
  {
    if(rc == MOSQ_ERR_ERRNO)
    {
      char err[1024];
#ifndef WIN32
      strerror_r(errno, err, 1024);
#else
      FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM, NULL, errno, 0, (LPTSTR)&err, 1024, NULL);
#endif
      QMessageBox::warning(nullptr, tr("MQTT Client"),
                           tr("Error: %1").arg(err),  QMessageBox::Ok);
    }
    else
    {
      QMessageBox::warning(nullptr, tr("MQTT Client"),
                           tr("Unable to connect (%1)").arg(mosquitto_strerror(rc)),
                           QMessageBox::Ok);
    }
    return false;
  }

  _running = true;

  mosquitto_loop_start(_mosq);

  return _running;
}

void DataStreamMQTT::shutdown()
{
  if( _running )
  {
    mosquitto_disconnect(_mosq);
    mosquitto_loop_stop(_mosq, true);
    mosquitto_destroy(_mosq);
    _mosq = nullptr;
    _disconnection_done = false;
    _running = false;
    _parsers.clear();
    dataMap().clear();
  }
}

bool DataStreamMQTT::isRunning() const
{
  return _running;
}



