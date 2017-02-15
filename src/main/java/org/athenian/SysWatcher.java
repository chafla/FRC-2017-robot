package org.athenian;

import org.athenian.args.ServerArgs;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttException;

import static java.lang.String.format;

public class SysWatcher {

    public static void main(final String[] argv) {

        final ServerArgs cliArgs = new ServerArgs();
        cliArgs.parseArgs(SysWatcher.class.getName(), argv);

        final String mqtt_hostname = Utils.getMqttHostname(cliArgs.mqtt_arg);
        final int mqtt_port = Utils.getMqttPort(cliArgs.mqtt_arg);

        final MqttClient client = Utils.createMqttClient(mqtt_hostname, mqtt_port, new BaseMqttCallback());
        if (client != null) {
            try {
                client.subscribe("$SYS/#",
                                 0,
                                 (topic, msg) -> System.out.println(format("%s : %s", topic, new String(msg.getPayload()))));
            }
            catch (MqttException e) {
                System.out.println(format("Unable to subscribe to system topics [%s]", e.getMessage()));
            }
        }
    }
}
