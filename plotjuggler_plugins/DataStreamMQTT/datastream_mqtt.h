#ifndef DATASTREAM_MQTT_H
#define DATASTREAM_MQTT_H

#include <QDialog>
#include <QtPlugin>
#include <QTimer>
#include <QThread>
#include "PlotJuggler/datastreamer_base.h"
#include "PlotJuggler/messageparser_base.h"
#include "ui_datastream_mqtt.h"

#include <mosquitto.h>

using namespace PJ;

struct MosquittoConfig {
  std::string id;
  std::string id_prefix;
  int protocol_version;
  int keepalive;
  std::string host;
  int port;
  int qos;
  bool retain;
  std::string bind_address;
#ifdef WITH_SRV
  bool use_srv;
#endif
  unsigned int max_inflight;
  std::string username;
  std::string password;
  std::string will_topic;
  std::string will_payload;
  int will_qos;
  bool will_retain;
#ifdef WITH_TLS
  std::string cafile;
  std::string capath;
  std::string certfile;
  std::string keyfile;
  std::string ciphers;
  bool insecure;
  std::string tls_version;
#  ifdef WITH_TLS_PSK
  std::string psk;
  std::string psk_identity;
#  endif
#endif
  bool clean_session; /* sub */
  std::vector<std::string> topics; /* sub */
  bool no_retain; /* sub */
  std::vector<std::string> filter_outs; /* sub */
  bool verbose; /* sub */
  bool eol; /* sub */
  int msg_count; /* sub */
#ifdef WITH_SOCKS
  std::string socks5_host;
  int socks5_port;
  std::string socks5_username;
  std::string socks5_password;
#endif
  mosquitto_property *connect_props;
  mosquitto_property *subscribe_props;
  mosquitto_property *unsubscribe_props;
  mosquitto_property *disconnect_props;
  mosquitto_property *will_props = nullptr;
};


class DataStreamMQTT : public PJ::DataStreamer
{
  Q_OBJECT
  Q_PLUGIN_METADATA(IID "facontidavide.PlotJuggler3.DataStreamer")
  Q_INTERFACES(PJ::DataStreamer)

public:
  DataStreamMQTT();

  ~DataStreamMQTT() override;

  virtual bool start(QStringList*) override;

  virtual void shutdown() override;

  virtual bool isRunning() const override;

  virtual const char* name() const override
  {
    return "MQTT Subscriber (Mosquitto)";
  }

  virtual bool isDebugPlugin() override
  {
    return false;
  }

  std::pair<QAction*, int> notificationAction() override
  {
    return { _notification_action, _failed_parsing };
  }

  bool _disconnection_done;
  bool _subscribed;
  bool _finished;
  bool _running;

  std::unordered_map<std::string, PJ::MessageParserPtr> _parsers;

  struct mosquitto *_mosq = nullptr;
  MosquittoConfig _config;

  std::thread _mqtt_thread;
  QString _protocol;

  QAction* _notification_action;
  int _failed_parsing = 0;

private slots:


};


#endif // DATASTREAM_MQTT_H
