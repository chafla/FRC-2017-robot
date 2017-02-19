package org.usfirst.frc.team852.robot;

import org.eclipse.paho.client.mqttv3.MqttClient;

@FunctionalInterface
public interface SubscribeAction {
    void subscribe(MqttClient mqttClient);
}
