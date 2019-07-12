# bond timeout issue

[`nodelet`][ros-nodelet] works with the manager-children structure, and [`bond`][ros-bond]s are formed between the manager and a child to transmit heartbeat data between the 2. `bond` is essentially a templated class to create publishers and subscribers for the `/<manager>/bond` topic. Definitions related to this analysis in the `Bond` class include:

- the `Bond` class defines timers for different purposes. The timer related to this issue is the `connect_timer_` variable
- the `Bond` class initializes all timers with default timeout values. The default timeout value for `connect_timer_` is `bond::Constants::DEFAULT_CONNECT_TIMEOUT`, which is `10.0` seconds by default
- the `Bond` class defines both `ros::Subscriber sub_` and `ros::Publisher pub_`, which are initialized upon `Bond::start()`

When a nodelet requests the manager to load code, a bond (if requested) will also be attempted to be formed. During this process, the nodelet manager and the child talk in the following steps:

1. child sends `/load_nodelet` service request to manager
2. manager receives `/load_nodelet` service request and loads nodelet code
3. manager process creates bond by calling `bond->start()`, and starts sending out heartbeat periodically
4. child receives (success) response from `/load_nodelet` service request
5. child process creates bond by calling `bond->start()`, and starts sending out heartbeat periodically

the bond class recognizes a connection between 2 process when its subscriber receives the first heartbeat signal from the other process. when the heartbeat signal takes too long to be received, it will mark the connection as timed out.

the following is a typical network log when the bond between a nodelet manager process and the nodelet process after removing the timeout. as we can see, the connection ultimately gets established after more than 10 seconds

```
[ INFO] [1562890405.970666100]: creating bond [ed2a422e-61dc-4904-88df-2f37e511b35d] in manager process
[ INFO] [1562890405.994268400]: starting bond [0609a515-48ac-4e2e-8a2c-6fd36ef697ff] in nodelet process
[ INFO] [1562890406.975367400]: ed2a422e-61dc-4904-88df-2f37e511b35d: heartbeat - 1
[ INFO] [1562890406.975669500]: ed2a422e-61dc-4904-88df-2f37e511b35d: bond status callback - (1562890406.975161500) ed2a422e-61dc-4904-88df-2f37e511b35d
[ INFO] [1562890407.000803900]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: bond status callback - (1562890406.975161500) ed2a422e-61dc-4904-88df-2f37e511b35d
[ INFO] [1562890407.101340700]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: heartbeat - 1
[ INFO] [1562890407.200162600]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: bond status callback - (1562890407.101267700) 0609a515-48ac-4e2e-8a2c-6fd36ef697ff
[ INFO] [1562890407.974825200]: ed2a422e-61dc-4904-88df-2f37e511b35d: heartbeat - 1
[ INFO] [1562890407.975015000]: ed2a422e-61dc-4904-88df-2f37e511b35d: bond status callback - (1562890407.974730900) ed2a422e-61dc-4904-88df-2f37e511b35d
[ INFO] [1562890408.000520200]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: bond status callback - (1562890407.974730900) ed2a422e-61dc-4904-88df-2f37e511b35d
[ INFO] [1562890408.000780400]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: heartbeat - 1
[ INFO] [1562890408.100311500]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: bond status callback - (1562890408.000661700) 0609a515-48ac-4e2e-8a2c-6fd36ef697ff
[ INFO] [1562890408.975287600]: ed2a422e-61dc-4904-88df-2f37e511b35d: heartbeat - 1
[ INFO] [1562890408.975501700]: ed2a422e-61dc-4904-88df-2f37e511b35d: bond status callback - (1562890408.975150700) ed2a422e-61dc-4904-88df-2f37e511b35d
[ INFO] [1562890409.000731900]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: bond status callback - (1562890408.975150700) ed2a422e-61dc-4904-88df-2f37e511b35d
[ INFO] [1562890409.001049400]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: heartbeat - 1
[ INFO] [1562890409.101234600]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: bond status callback - (1562890409.000942500) 0609a515-48ac-4e2e-8a2c-6fd36ef697ff
[ INFO] [1562890409.974813900]: ed2a422e-61dc-4904-88df-2f37e511b35d: heartbeat - 1
[ INFO] [1562890409.975078600]: ed2a422e-61dc-4904-88df-2f37e511b35d: bond status callback - (1562890409.974681600) ed2a422e-61dc-4904-88df-2f37e511b35d
[ INFO] [1562890410.001003300]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: bond status callback - (1562890409.974681600) ed2a422e-61dc-4904-88df-2f37e511b35d
[ INFO] [1562890410.001354200]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: heartbeat - 1
[ INFO] [1562890410.100709100]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: bond status callback - (1562890410.001208600) 0609a515-48ac-4e2e-8a2c-6fd36ef697ff
[ INFO] [1562890410.975656500]: ed2a422e-61dc-4904-88df-2f37e511b35d: heartbeat - 1
[ INFO] [1562890410.975812800]: ed2a422e-61dc-4904-88df-2f37e511b35d: bond status callback - (1562890410.975562800) ed2a422e-61dc-4904-88df-2f37e511b35d
[ INFO] [1562890411.000465900]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: bond status callback - (1562890410.975562800) ed2a422e-61dc-4904-88df-2f37e511b35d
[ INFO] [1562890411.099975500]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: heartbeat - 1
[ INFO] [1562890411.200872200]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: bond status callback - (1562890411.099921700) 0609a515-48ac-4e2e-8a2c-6fd36ef697ff
[ INFO] [1562890411.975869200]: ed2a422e-61dc-4904-88df-2f37e511b35d: heartbeat - 1
[ INFO] [1562890411.976046300]: ed2a422e-61dc-4904-88df-2f37e511b35d: bond status callback - (1562890411.975763300) ed2a422e-61dc-4904-88df-2f37e511b35d
[ INFO] [1562890412.000183100]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: bond status callback - (1562890411.975763300) ed2a422e-61dc-4904-88df-2f37e511b35d
[ INFO] [1562890412.100907600]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: heartbeat - 1
[ INFO] [1562890412.200875300]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: bond status callback - (1562890412.100867800) 0609a515-48ac-4e2e-8a2c-6fd36ef697ff
[ INFO] [1562890412.976709700]: ed2a422e-61dc-4904-88df-2f37e511b35d: heartbeat - 1
[ INFO] [1562890412.976921500]: ed2a422e-61dc-4904-88df-2f37e511b35d: bond status callback - (1562890412.976630700) ed2a422e-61dc-4904-88df-2f37e511b35d
[ INFO] [1562890413.000973600]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: bond status callback - (1562890412.976630700) ed2a422e-61dc-4904-88df-2f37e511b35d
[ INFO] [1562890413.100695900]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: heartbeat - 1
[ INFO] [1562890413.200892500]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: bond status callback - (1562890413.100656800) 0609a515-48ac-4e2e-8a2c-6fd36ef697ff
[ INFO] [1562890413.975151300]: ed2a422e-61dc-4904-88df-2f37e511b35d: heartbeat - 1
[ INFO] [1562890413.975465600]: ed2a422e-61dc-4904-88df-2f37e511b35d: bond status callback - (1562890413.974932100) ed2a422e-61dc-4904-88df-2f37e511b35d
[ INFO] [1562890414.000426200]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: bond status callback - (1562890413.974932100) ed2a422e-61dc-4904-88df-2f37e511b35d
[ INFO] [1562890414.100246400]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: heartbeat - 1
[ INFO] [1562890414.201452000]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: bond status callback - (1562890414.100197000) 0609a515-48ac-4e2e-8a2c-6fd36ef697ff
[ INFO] [1562890414.976412200]: ed2a422e-61dc-4904-88df-2f37e511b35d: heartbeat - 1
[ INFO] [1562890414.976653500]: ed2a422e-61dc-4904-88df-2f37e511b35d: bond status callback - (1562890414.976246100) ed2a422e-61dc-4904-88df-2f37e511b35d
[ INFO] [1562890415.000773000]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: bond status callback - (1562890414.976246100) ed2a422e-61dc-4904-88df-2f37e511b35d
[ INFO] [1562890415.001125100]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: heartbeat - 1
[ INFO] [1562890415.100916600]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: bond status callback - (1562890415.001086300) 0609a515-48ac-4e2e-8a2c-6fd36ef697ff
[ INFO] [1562890415.975091500]: ed2a422e-61dc-4904-88df-2f37e511b35d: heartbeat - 1
[ INFO] [1562890415.975380900]: ed2a422e-61dc-4904-88df-2f37e511b35d: bond status callback - (1562890415.974927000) ed2a422e-61dc-4904-88df-2f37e511b35d
[ INFO] [1562890416.000446900]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: bond status callback - (1562890415.974927000) ed2a422e-61dc-4904-88df-2f37e511b35d
[ INFO] [1562890416.100609400]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: heartbeat - 1
[ INFO] [1562890416.200165700]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: bond status callback - (1562890416.100518600) 0609a515-48ac-4e2e-8a2c-6fd36ef697ff
[ INFO] [1562890416.974734700]: ed2a422e-61dc-4904-88df-2f37e511b35d: heartbeat - 1
[ INFO] [1562890416.974915000]: ed2a422e-61dc-4904-88df-2f37e511b35d: bond status callback - (1562890416.974628100) ed2a422e-61dc-4904-88df-2f37e511b35d
[ INFO] [1562890417.000062700]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: bond status callback - (1562890416.974628100) ed2a422e-61dc-4904-88df-2f37e511b35d
[ INFO] [1562890417.100349300]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: heartbeat - 1
[ INFO] [1562890417.200339600]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: bond status callback - (1562890417.100282800) 0609a515-48ac-4e2e-8a2c-6fd36ef697ff
[ INFO] [1562890417.977415300]: ed2a422e-61dc-4904-88df-2f37e511b35d: heartbeat - 1
[ INFO] [1562890417.977700000]: ed2a422e-61dc-4904-88df-2f37e511b35d: bond status callback - (1562890417.977267800) ed2a422e-61dc-4904-88df-2f37e511b35d
[ INFO] [1562890418.000681500]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: bond status callback - (1562890417.977267800) ed2a422e-61dc-4904-88df-2f37e511b35d
[ INFO] [1562890418.000986500]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: heartbeat - 1
[ INFO] [1562890418.101075300]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: bond status callback - (1562890418.000944100) 0609a515-48ac-4e2e-8a2c-6fd36ef697ff
[ INFO] [1562890418.975271900]: ed2a422e-61dc-4904-88df-2f37e511b35d: heartbeat - 1
[ INFO] [1562890418.999782000]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: bond status callback - (1562890418.975120700) ed2a422e-61dc-4904-88df-2f37e511b35d
[ INFO] [1562890419.063147200]: ed2a422e-61dc-4904-88df-2f37e511b35d: bond status callback - (1562890418.975120700) ed2a422e-61dc-4904-88df-2f37e511b35d
[ INFO] [1562890419.100688200]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: heartbeat - 1
[ INFO] [1562890419.100421200]: ed2a422e-61dc-4904-88df-2f37e511b35d: bond status callback - (1562890419.100563900) 0609a515-48ac-4e2e-8a2c-6fd36ef697ff
[ INFO] [1562890419.200455000]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: bond status callback - (1562890419.100563900) 0609a515-48ac-4e2e-8a2c-6fd36ef697ff
[ INFO] [1562890419.975241800]: ed2a422e-61dc-4904-88df-2f37e511b35d: heartbeat - 1
[ INFO] [1562890419.975373100]: ed2a422e-61dc-4904-88df-2f37e511b35d: bond status callback - (1562890419.975075500) ed2a422e-61dc-4904-88df-2f37e511b35d
[ INFO] [1562890420.000584600]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: bond status callback - (1562890419.975075500) ed2a422e-61dc-4904-88df-2f37e511b35d
[ INFO] [1562890420.100615700]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: heartbeat - 1
[ INFO] [1562890420.100336000]: ed2a422e-61dc-4904-88df-2f37e511b35d: bond status callback - (1562890420.100524300) 0609a515-48ac-4e2e-8a2c-6fd36ef697ff
[ INFO] [1562890420.200336100]: 0609a515-48ac-4e2e-8a2c-6fd36ef697ff: bond status callback - (1562890420.100524300) 0609a515-48ac-4e2e-8a2c-6fd36ef697ff
```

to simplify the explanation, we use the following aliases for the bonds and their publishers and subscribers:

- `bond [ed2a422e-61dc-4904-88df-2f37e511b35d]` created in the manager process as `b1`, its publisher as `b1-pub` and subscriber as `b1-sub`
- `bond [0609a515-48ac-4e2e-8a2c-6fd36ef697ff]` created in the nodelet process as `b2`, its publisher as `b2-pub` and subscriber as `b2-sub`

what we have noticed is that even if `b2` (in the child process) has started to send out heartbeat signal (its own subscriber-callbacks are getting called), `b1` is not getting any heartbeat callback at all for the first few seconds (around 14 seconds). The log indicates:

- both bonds have been properly initialized
- publishers and subscribers of both bonds have also been properly initialized
- `b1-sub` is not getting signal from `b2-pub`
- `b2-sub` is getting signal from `b1-pub` as expected

Noticing that `b1` with its publisher and subscriber are created earlier than `b2` and its publisher and subscriber, we can conclude that `b1-pub` is created earlier than `b2-sub` (this is because publisher and subscriber creation code is blocking). Meantime, however, `b2-pub` is created later than `b1-sub`. Would this be the reason for the behavior above? Searching online would lead us to [this question][ros_answers-msgs_not_received_if_publisher_starts_after_subscriber] that has been discussed on answers.ros. The root cause for this is that when a publisher is created later than a subscriber, the subscriber takes a long time to establish a connection with the newly emerged publisher.

As explained in the "Node Environment Variables" section of [this ROS documentation][ros-environment_variables], when a machine has more than one network name this problem starts to emerge. To work around it, use the `ROS_IP` or `ROS_HOSTNAME` environment variable to specify a designated connection. The network log proves that it works:

```
[ INFO] [1562890762.216079700]: creating bond [469d6a4a-6f6c-40e9-a35a-05526812d10e] in manager process
[ INFO] [1562890763.272645100]: starting bond [260dff49-6afa-47a3-b288-fb560cd53edc] in nodelet process
[ INFO] [1562890763.220602400]: 469d6a4a-6f6c-40e9-a35a-05526812d10e: heartbeat - 1
[ INFO] [1562890763.220870700]: 469d6a4a-6f6c-40e9-a35a-05526812d10e: bond status callback - (1562890763.220382500) 469d6a4a-6f6c-40e9-a35a-05526812d10e
[ INFO] [1562890764.282294900]: 260dff49-6afa-47a3-b288-fb560cd53edc: bond status callback - (1562890763.220382500) 469d6a4a-6f6c-40e9-a35a-05526812d10e
[ INFO] [1562890764.382758000]: 260dff49-6afa-47a3-b288-fb560cd53edc: heartbeat - 1
[ INFO] [1562890763.382176600]: 469d6a4a-6f6c-40e9-a35a-05526812d10e: bond status callback - (1562890764.382646800) 260dff49-6afa-47a3-b288-fb560cd53edc
[ INFO] [1562890764.482311800]: 260dff49-6afa-47a3-b288-fb560cd53edc: bond status callback - (1562890764.382646800) 260dff49-6afa-47a3-b288-fb560cd53edc
[ INFO] [1562890764.220152000]: 469d6a4a-6f6c-40e9-a35a-05526812d10e: heartbeat - 1
[ INFO] [1562890764.220486600]: 469d6a4a-6f6c-40e9-a35a-05526812d10e: bond status callback - (1562890764.220003500) 469d6a4a-6f6c-40e9-a35a-05526812d10e
[ INFO] [1562890765.282785900]: 260dff49-6afa-47a3-b288-fb560cd53edc: bond status callback - (1562890764.220003500) 469d6a4a-6f6c-40e9-a35a-05526812d10e
[ INFO] [1562890765.382817700]: 260dff49-6afa-47a3-b288-fb560cd53edc: heartbeat - 1
[ INFO] [1562890764.382367100]: 469d6a4a-6f6c-40e9-a35a-05526812d10e: bond status callback - (1562890765.382671600) 260dff49-6afa-47a3-b288-fb560cd53edc
[ INFO] [1562890765.482481400]: 260dff49-6afa-47a3-b288-fb560cd53edc: bond status callback - (1562890765.382671600) 260dff49-6afa-47a3-b288-fb560cd53edc
[ INFO] [1562890765.221731600]: 469d6a4a-6f6c-40e9-a35a-05526812d10e: heartbeat - 1
[ INFO] [1562890765.222049500]: 469d6a4a-6f6c-40e9-a35a-05526812d10e: bond status callback - (1562890765.221565600) 469d6a4a-6f6c-40e9-a35a-05526812d10e
[ INFO] [1562890766.282347400]: 260dff49-6afa-47a3-b288-fb560cd53edc: bond status callback - (1562890765.221565600) 469d6a4a-6f6c-40e9-a35a-05526812d10e
[ INFO] [1562890766.382124800]: 260dff49-6afa-47a3-b288-fb560cd53edc: heartbeat - 1
[ INFO] [1562890765.381757300]: 469d6a4a-6f6c-40e9-a35a-05526812d10e: bond status callback - (1562890766.381970500) 260dff49-6afa-47a3-b288-fb560cd53edc
[ INFO] [1562890766.483556400]: 260dff49-6afa-47a3-b288-fb560cd53edc: bond status callback - (1562890766.381970500) 260dff49-6afa-47a3-b288-fb560cd53edc
[ INFO] [1562890766.220679800]: 469d6a4a-6f6c-40e9-a35a-05526812d10e: heartbeat - 1
[ INFO] [1562890766.221091400]: 469d6a4a-6f6c-40e9-a35a-05526812d10e: bond status callback - (1562890766.220478900) 469d6a4a-6f6c-40e9-a35a-05526812d10e
[ INFO] [1562890767.282366100]: 260dff49-6afa-47a3-b288-fb560cd53edc: bond status callback - (1562890766.220478900) 469d6a4a-6f6c-40e9-a35a-05526812d10e
[ INFO] [1562890767.382669700]: 260dff49-6afa-47a3-b288-fb560cd53edc: heartbeat - 1
[ INFO] [1562890766.382365400]: 469d6a4a-6f6c-40e9-a35a-05526812d10e: bond status callback - (1562890767.382425100) 260dff49-6afa-47a3-b288-fb560cd53edc
[ INFO] [1562890767.482913600]: 260dff49-6afa-47a3-b288-fb560cd53edc: bond status callback - (1562890767.382425100) 260dff49-6afa-47a3-b288-fb560cd53edc
[ INFO] [1562890767.220318200]: 469d6a4a-6f6c-40e9-a35a-05526812d10e: heartbeat - 1
[ INFO] [1562890767.220589500]: 469d6a4a-6f6c-40e9-a35a-05526812d10e: bond status callback - (1562890767.220160300) 469d6a4a-6f6c-40e9-a35a-05526812d10e
[ INFO] [1562890768.282961200]: 260dff49-6afa-47a3-b288-fb560cd53edc: bond status callback - (1562890767.220160300) 469d6a4a-6f6c-40e9-a35a-05526812d10e
[ INFO] [1562890768.383043300]: 260dff49-6afa-47a3-b288-fb560cd53edc: heartbeat - 1
[ INFO] [1562890767.382516700]: 469d6a4a-6f6c-40e9-a35a-05526812d10e: bond status callback - (1562890768.382942700) 260dff49-6afa-47a3-b288-fb560cd53edc
[ INFO] [1562890768.483124600]: 260dff49-6afa-47a3-b288-fb560cd53edc: bond status callback - (1562890768.382942700) 260dff49-6afa-47a3-b288-fb560cd53edc
[ INFO] [1562890768.219913400]: 469d6a4a-6f6c-40e9-a35a-05526812d10e: heartbeat - 1
[ INFO] [1562890768.220194100]: 469d6a4a-6f6c-40e9-a35a-05526812d10e: bond status callback - (1562890768.219739700) 469d6a4a-6f6c-40e9-a35a-05526812d10e
[ INFO] [1562890769.283423200]: 260dff49-6afa-47a3-b288-fb560cd53edc: bond status callback - (1562890768.219739700) 469d6a4a-6f6c-40e9-a35a-05526812d10e
[ INFO] [1562890769.382575200]: 260dff49-6afa-47a3-b288-fb560cd53edc: heartbeat - 1
[ INFO] [1562890768.382076400]: 469d6a4a-6f6c-40e9-a35a-05526812d10e: bond status callback - (1562890769.382510900) 260dff49-6afa-47a3-b288-fb560cd53edc
[ INFO] [1562890769.481919300]: 260dff49-6afa-47a3-b288-fb560cd53edc: bond status callback - (1562890769.382510900) 260dff49-6afa-47a3-b288-fb560cd53edc
```

<!-- external links -->
[ros-bond]: http://wiki.ros.org/bond
[ros-environment_variables]: http://wiki.ros.org/ROS/EnvironmentVariables
[ros-nodelet]: http://wiki.ros.org/nodelet
[ros_answers-msgs_not_received_if_publisher_starts_after_subscriber]: https://answers.ros.org/question/10171/ros-msgs-not-received-if-publisher-starts-after-subscriber/
