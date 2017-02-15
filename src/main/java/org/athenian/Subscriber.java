package org.athenian;

import org.athenian.args.TopicArgs;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttException;

import java.nio.ByteBuffer;

import static java.lang.String.format;

public class Subscriber {

    public static void main(final String[] argv) throws InterruptedException {
        final TopicArgs cliArgs = new TopicArgs();
        cliArgs.parseArgs(Subscriber.class.getName(), argv);

        final String mqtt_hostname = Utils.getMqttHostname(cliArgs.mqtt_arg);
        final int mqtt_port = Utils.getMqttPort(cliArgs.mqtt_arg);

        final MqttClient client = Utils.createMqttClient(mqtt_hostname, mqtt_port, new BaseMqttCallback());
        if (client != null) {
            try {
                client.subscribe(cliArgs.mqtt_topic,
                                 (topic, msg) -> System.out.println(format("%s : %d",
                                                                           topic, ByteBuffer.wrap(msg.getPayload()).getInt())));
            }
            catch (MqttException e) {
                System.out.println(format("Unable to publish data to %s [%s]", cliArgs.mqtt_topic, e.getMessage()));
            }
        }
    }
}
