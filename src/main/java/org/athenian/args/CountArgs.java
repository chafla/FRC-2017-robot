package org.athenian.args;

import com.beust.jcommander.Parameter;

public class CountArgs extends TopicArgs {
    @Parameter(names = {"-c", "--count"}, description = "Number of messages to publish")
    public int mqtt_count = 1000;
}
