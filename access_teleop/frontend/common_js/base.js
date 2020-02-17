/**
 * Created by Xinyi on 05/09/19.
 */

Base = function (ros) {

    var baseTopic = new ROSLIB.Topic({
        ros: ros,
        name: 'cmd_vel',
        messageType: 'geometry_msgs/Twist'
    });

    this.adv = function() {
        baseTopic.advertise();
    };

    this.unadv = function() {
        baseTopic.unadvertise();
    };
};

function move(direction, topic) {
    var lv = 0;
    var rv = 0;
    if (direction == "forward") {
        lv = 0.5;
    } else if (direction == "back") {
        lv = -0.5;
    } else if (direction == "left") {
        rv = 1;
    } else {
        rv = -1;
    }
    topic.publish({
        linear : {
            x: lv, // Set positive or negative meters/s to drive
            y: 0,
            z: 0
        },
        angular : {
            x: 0,
            y: 0,
            z: rv // Set rads/s to turn
        }
    });
}