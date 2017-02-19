package org.usfirst.frc.team852.robot;

import org.athenian.BaseMqttCallback;
import org.athenian.Utils;
import org.eclipse.paho.client.mqttv3.MqttClient;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import static org.athenian.Utils.createMqttClient;

public class MqttReconnect {

    private final ExecutorService executor = Executors.newFixedThreadPool(1);
    private final AtomicReference<MqttClient> mqttClientRef = new AtomicReference<>();
    private final AtomicBoolean stopped = new AtomicBoolean(false);

    private final String mqtt_hostname;
    private final int mqtt_port;
    private final int connectionTimeout;
    private final SubscribeAction subscribeAction;

    public MqttReconnect(String mqtt_hostname,
                         int mqtt_port,
                         int connectionTimeout,
                         SubscribeAction subscribeAction) {
        this.mqtt_hostname = mqtt_hostname;
        this.mqtt_port = mqtt_port;
        this.connectionTimeout = connectionTimeout;
        this.subscribeAction = subscribeAction;
    }

    public void start() {
        this.executor.submit(
                () -> {
                    while (!stopped.get()) {
                        try {
                            if (mqttClientRef.get() == null) {
                                mqttClientRef.set(
                                        createMqttClient(mqtt_hostname,
                                                         mqtt_port,
                                                         false,
                                                         connectionTimeout,
                                                         new BaseMqttCallback() {
                                                             @Override
                                                             public void connectionLost(Throwable throwable) {
                                                                 super.connectionLost(throwable);
                                                                 mqttClientRef.set(null);
                                                                 synchronized (mqttClientRef) {
                                                                     System.out.println("MqttReconnect calling notifyAll()");
                                                                     mqttClientRef.notifyAll();
                                                                 }
                                                             }
                                                         }));
                                if (mqttClientRef.get() == null) {
                                    System.out.println("Error connecting to MQTT broker");
                                    Utils.sleepSecs(1);
                                } else {
                                    // Call SubscribeAction interface to subscribe to topics
                                    subscribeAction.subscribe(mqttClientRef.get());
                                }
                            } else {
                                synchronized (mqttClientRef) {
                                    try {
                                        System.out.println("MqttReconnect calling wait()");
                                        mqttClientRef.wait(1000);
                                        System.out.println("MqttReconnect returned from wait()");
                                    }
                                    catch (InterruptedException e) {
                                        System.out.println("MqttReconnect interrupted from wait()");
                                        e.printStackTrace();
                                    }
                                }
                            }
                        }
                        catch (Throwable e) {
                            System.out.println(String.format("Error in MqttReconnect thread [%s - %s]",
                                                             e.getClass().getSimpleName(),
                                                             e.getMessage()));
                            e.printStackTrace();
                        }
                    }

                }
        );
    }

    public MqttClient getMqttClient() {
        return this.mqttClientRef.get();
    }

    public void stop() {
        stopped.set(true);
    }
}
