/**
 * Created by timadamson on 8/22/17.
 */

/**
 * An App self adds 3 camera streams, and the controlers to move the robot
 */

App = function () {


    // Set self to be this so that you can add variables to this inside a callback
    var self = this;

    // Set up ros
    let websocketUrl = (function() {
        let hostname = window.location.hostname;
        let protocol = 'ws:';
        if (window.location.protocol === 'https:') {
            protocol = 'wss:';
        }
        return protocol + '//' + hostname + ':9090';
    })();
    this.ros = new ROSLIB.Ros({
        url : websocketUrl
    });

    this.arm = new Arm(this.ros);
    this.gripper = new Gripper(this.ros);
    this.cloudFreezer = new CloudFreezer(this.ros);
    this.base = new Base(this.ros);
    this.wristRoller = new WristRoller(this.ros);
    this.head = new Head(this.ros);

    this.ros.on('error', function(error) {
        console.log('Error connecting to websocket server.');
    });

    this.advertiseAll = function() {
        self.arm.adv();
        self.cloudFreezer.adv();
        self.base.adv();
        self.wristRoller.adv();
        self.head.adv();

        console.log("Advertised all publishers");
    };

    this.unadvertiseAll = function() {
        self.arm.unadv();
        self.cloudFreezer.unadv();
        self.base.unadv();
        self.wristRoller.unadv();
        self.head.unadv();

        console.log("Unadvertised all publishers");
    };


    // Set up the gripper event handlers
    // Calls itself after definition
    this.initRightClickGripper = function () {
        var arm_div = document.querySelectorAll('.js_arm_div');
        arm_div.forEach(function(element){
            element.addEventListener('contextmenu', function(ev){
                ev.preventDefault();
                if(self.gripper.getCurrentPosition() == self.gripper.PositionEnum.CLOSED ||
                    self.gripper.getCurrentPosition() == self.gripper.PositionEnum.PARTLY_CLOSED) {
                    self.gripper.open();
                }
                else {
                    self.gripper.close();
                }
               return false;
            }, false);
        });
    };

    this.addCloudFreezer = function(){
        var feedback = document.querySelector("#feedback");

        var freezeButton = document.createElement("button");
        freezeButton.innerHTML = "Freeze Point Cloud";
        freezeButton.onclick = this.cloudFreezer.freezeCloud;

        var unfreezeButton = document.createElement("button");
        unfreezeButton.innerHTML = "Real Time Point Cloud";
        unfreezeButton.onclick = this.cloudFreezer.unfreezeCloud;

        feedback.appendChild(freezeButton);
        feedback.appendChild(unfreezeButton);
    };

    this.moveLine = function (x1,y1, x2, y2, camera_name) {
        if(camera_name) {
            var line = document.querySelector("#" + camera_name + "  svg line");
            line.setAttribute('x1', x1);
            line.setAttribute('y1', y1);
            line.setAttribute('x2', x2);
            line.setAttribute('y2', y2);
        }
    };

    this.moveArrow = function (x1,y1, x2, y2, camera_name) {
        if(camera_name) {
            var polyline = document.querySelector("#" + camera_name + "  svg polyline");
            polyline.setAttribute('points', `${x1},${y1} ${x2},${y2}`);
        }
    };

    // This is the double slider for the gripper
    $(document).arrive("#slider-range", function () {
        $( function() {
            $slider = $( "#slider-range" ); // This will create the slider jQuery object as soon as the element is ready
            app.gripper.gripperGUI = new GripperGUI(app.gripper); // This has to be here, because otherwise the element wil not be loaded
        });
    });

    // Adds 3 canvas image streams
    // --------------------------------------------------------------------------------
    // Dynamic Canvas Sizes
    // var camCanvas = document.getElementById("cam1");

    this.cameraWidth = (window.innerWidth - 20) / 3;
    this.cameraHeight = "480";

    this.backendCameraWidth = "640";
    this.backendCameraHeight = "480";

    document.getElementById("camera1").style.height = this.cameraHeight + 'px';
    document.getElementById("camera1").style.width = this.cameraWidth + 'px';

    document.getElementById("camera2").style.height = this.cameraHeight + 'px';
    document.getElementById("camera2").style.width = this.cameraWidth + 'px';

    document.getElementById("camera3").style.height = this.cameraHeight + 'px';
    document.getElementById("camera3").style.width = this.cameraWidth + 'px';

    // Dynamic Canvas Sizes

    var rightForearmWidth=this.cameraWidth;
    var rightForearmHeight=this.cameraHeight;

    var headWidth=this.cameraWidth;
    var headHeight=this.cameraHeight;

    var frontWidth=this.cameraWidth;
    var frontHeight=this.cameraHeight;


    // var elmntmjpegLeftForearm = document.getElementById("camera3");
    // var leftForearmWidth=elmntmjpegLeftForearm.clientWidth;
    // var leftForearmHeight=leftForearmWidth;

    // Create the right forearm viewer.
    var forearmRViewer = new MJPEGCANVAS.Viewer({
        divID : 'camera1',
        host : 'localhost',
        width : rightForearmWidth,
        height : rightForearmHeight,
        topic : '/rviz1/camera1/image'
    });

    // Create the head viewer.
    var headViewer = new MJPEGCANVAS.Viewer({
        divID : 'camera2',
        host : 'localhost',
        width : headWidth,
        height : headHeight,
        topic : '/rviz1/camera2/image'
    });

    // // Create the left forearm viewer.
    // var forearmLViewer = new MJPEGCANVAS.Viewer({
    //     divID : 'camera3',
    //     host : 'localhost',
    //     width : leftForearmWidth,
    //     height : leftForearmHeight,
    //     topic : '/head_camera/rgb/image_raw'
    // });

    // Create the fromt viewer.
    var frontViewer = new MJPEGCANVAS.Viewer({
        divID : 'camera3',
        host : 'localhost',
        width : frontWidth,
        height : frontHeight,
        topic : '/rviz1/camera3/image'
    });

    // ------------------------------------------------------------------------------------------

    init_flag = false;


};



// window.onbeforeunload = function(event) {}
//     return confirm("Confirm refresh");
// };
