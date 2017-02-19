package org.athenian;

import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.MqttCallbackExtended;
import org.eclipse.paho.client.mqttv3.MqttMessage;

public class BaseMqttCallback implements MqttCallbackExtended {

    @Override
    public void connectionLost(Throwable throwable) {
        System.out.println(String.format("Connection to MQTT server lost [%s - %s]",
                                         throwable.getClass().getSimpleName(),
                                         throwable.getMessage()));
    }

    @Override
    public void connectComplete(boolean reconnect, String url) {
        System.out.println(String.format("%s to MQTT server %s",
                                         reconnect ? "Reconnected" : "Initial connection",
                                         url));
    }

    @Override
    public void messageArrived(String topic, MqttMessage msg) throws Exception {
        // Empty
    }

    @Override
    public void deliveryComplete(IMqttDeliveryToken iMqttDeliveryToken) {
        // Empty
    }

}
