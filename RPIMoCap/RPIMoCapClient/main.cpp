#include "rpicamera.h"
#include "rpimocapclient.h"

#include <mqttsubscriber.h>
#include <mqttpublisher.h>

#include <QCoreApplication>
#include <QHostAddress>
#include <QNetworkInterface>

int main(int argc, char *argv[])
{
    mosqpp::lib_init();

    QCoreApplication a(argc, argv);

    RPIMoCap::MQTTSettings settings;
    settings.IPAddress = "192.168.0.185";

    RPIMoCap::MQTTSubscriber subscriber("camera1trigger", "/trigger", settings);
    RPIMoCap::MQTTPublisher publisher("camera1linesPub","/lines",settings);

    RPIMoCapClient client;
    QObject::connect(&subscriber, &RPIMoCap::MQTTSubscriber::messageReceived,&client, &RPIMoCapClient::trigger);
    QObject::connect(&client, &RPIMoCapClient::linesSerialized, &publisher, &RPIMoCap::MQTTPublisher::publishData);

    auto ret = a.exec();

    mosqpp::lib_cleanup();

    return ret;
}
