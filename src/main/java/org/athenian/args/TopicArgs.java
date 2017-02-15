package org.athenian.args;

import com.beust.jcommander.Parameter;

public class TopicArgs extends ServerArgs {
    @Parameter(names = {"-t", "--topic"}, required = true, description = "MQTT topic")
    public String mqtt_topic;
}
