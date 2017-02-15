package org.athenian;

import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.MqttCallback;
import org.eclipse.paho.client.mqttv3.MqttMessage;

public class BaseMqttCallback implements MqttCallback {

    @Override
    public void connectionLost(Throwable throwable) {
        System.out.println("Connection to MQTT server lost");
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
