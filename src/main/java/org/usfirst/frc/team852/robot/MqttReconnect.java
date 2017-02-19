package org.usfirst.frc.team852.robot;

import org.athenian.BaseMqttCallback;
import org.athenian.Utils;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import static java.lang.String.format;
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
        final String url = format("tcp://%s:%d", mqtt_hostname, mqtt_port);

        final MqttConnectOptions opts = new MqttConnectOptions();
        opts.setCleanSession(true);
        opts.setAutomaticReconnect(true);
        opts.setConnectionTimeout(30);

        this.executor.submit(
                () -> {
                    while (!stopped.get()) {
                        try {
                            if (mqttClientRef.get() == null) {
                                mqttClientRef.set(
                                        createMqttClient(url,
                                                         new BaseMqttCallback() {
                                                             @Override
                                                             public void connectionLost(Throwable throwable) {
                                                                 super.connectionLost(throwable);
                                                                 // Null out value on disconnect
                                                                 mqttClientRef.set(null);
                                                                 synchronized (mqttClientRef) {
                                                                     System.out.println("MqttReconnect calling notifyAll()");
                                                                     mqttClientRef.notifyAll();
                                                                 }
                                                             }
                                                         }));
                                if (mqttClientRef.get() == null) {
                                    System.out.println("Error creating to MQTT client");
                                    Utils.sleepSecs(1);
                                } else {
                                    try {
                                        System.out.println(format("Connecting to MQTT broker at %s...", url));
                                        this.mqttClientRef.get().connect(opts);
                                        System.out.println(format("Connected to %s", url));

                                        // Call SubscribeAction interface to subscribe to topics
                                        subscribeAction.subscribe(mqttClientRef.get());
                                    }
                                    catch (MqttException e) {
                                        System.out.println(format("Cannot connect to MQTT broker at %s [%s]",
                                                                  url,
                                                                  e.getMessage()));
                                        e.printStackTrace();
                                        this.mqttClientRef.set(null);
                                    }
                                }
                            } else {
                                // Call wait() if value is already set
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
