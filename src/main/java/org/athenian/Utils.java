package org.athenian;

import org.eclipse.paho.client.mqttv3.*;
import org.eclipse.paho.client.mqttv3.persist.MemoryPersistence;

import static java.lang.String.format;

public class Utils {

    public static String getMqttHostname(String val) {
        return val.contains(":") ? val.substring(0, val.indexOf(":")) : val;
    }

    public static int getMqttPort(String s) {
        return s.contains(":") ? Integer.parseInt(s.substring(s.indexOf(":") + 1, s.length())) : 1883;
    }

    public static void sleepSecs(final long secs) {
        try {
            Thread.sleep(secs * 1000);
        }
        catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public static MqttClient createMqttClient(final String url,
                                              final MqttCallbackExtended callback) {

        try {
            final MqttClient mqttClient = new MqttClient(url, MqttClient.generateClientId(), new MemoryPersistence());
            if (callback != null)
                mqttClient.setCallback(callback);
            return mqttClient;
        }
        catch (MqttException e) {
            System.out.println(format("Cannot create MQTT client [%s - %s]",
                                      e.getClass().getSimpleName(),
                                      e.getMessage()));
            e.printStackTrace();
            return null;
        }
    }

    public static MqttAsyncClient createMqttAsyncClient(final String mqtt_hostname,
                                                        final int mqtt_port,
                                                        final boolean reconnect,
                                                        int connectionTimeout,
                                                        final MqttCallbackExtended callback) {

        final String url = format("tcp://%s:%d", mqtt_hostname, mqtt_port);
        try {
            final MqttAsyncClient mqttClient = new MqttAsyncClient(url, MqttClient.generateClientId(), new MemoryPersistence());
            if (callback != null)
                mqttClient.setCallback(callback);

            final MqttConnectOptions opts = new MqttConnectOptions();
            opts.setCleanSession(true);
            opts.setAutomaticReconnect(reconnect);
            opts.setConnectionTimeout(connectionTimeout);

            System.out.println(format("Connecting to MQTT server at %s...", url));
            mqttClient.connect(opts);

            System.out.println(format("Connected to %s", url));
            return mqttClient;
        }
        catch (MqttException e) {
            System.out.println(format("Cannot connect to MQTT server at %s [%s]", url, e.getMessage()));
            e.printStackTrace();
            return null;
        }
    }
}
