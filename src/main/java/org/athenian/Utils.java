package org.athenian;

import org.eclipse.paho.client.mqttv3.MqttCallbackExtended;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.persist.MemoryPersistence;

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

    public static MqttClient createMqttClient(final String mqtt_hostname,
                                              final int mqtt_port,
                                              final boolean reconnect,
                                              int connectionTimeout,
                                              final MqttCallbackExtended callback) {

        try {
            final MqttClient client = new MqttClient(String.format("tcp://%s:%d", mqtt_hostname, mqtt_port),
                                                     MqttClient.generateClientId(),
                                                     new MemoryPersistence());
            if (callback != null)
                client.setCallback(callback);

            final MqttConnectOptions opts = new MqttConnectOptions();
            opts.setCleanSession(true);
            opts.setAutomaticReconnect(reconnect);
            opts.setConnectionTimeout(connectionTimeout);

            System.out.println(String.format("Connecting to MQTT server at %s:%d...", mqtt_hostname, mqtt_port));
            client.connect(opts);

            System.out.println(String.format("Connected to %s:%d", mqtt_hostname, mqtt_port));
            return client;
        }
        catch (MqttException e) {
            e.printStackTrace();
            System.out.println(String.format("Cannot connect to MQTT server at: %s:%d [%s]",
                                             mqtt_hostname, mqtt_port, e.getMessage()));
            return null;
        }
    }
}
