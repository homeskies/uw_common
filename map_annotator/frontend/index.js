"use strict";

$(function() {
    const NS = 'http://www.w3.org/2000/svg';
    const HIGHLIGHT = "#ffc30b";
    const WHITE = "#ffffff";
    const DARK_GREY = "#666666";
    const RED = "#f10000";
    const BLUE = "#4c4cff";
    const YELLOW = "#ffd700";
    const DARK_YELLOW = "#998100";
    const GREEN = "#a7c44c";
    
    let circleRadius = 3.5;
    let lineWidth = 4;
    const LINE_LENGTH = 18;
    const INITIAL_REGION_SIZE = 50;
    const LABEL_PADDING = 20;
    const LABEL_FONT_SIZE = "10px";

    const UNIT_VECTOR = new ROSLIB.Vector3({x: 0, y: 1, z: 0});

    let self = this;

    let midpointX = 0;
    let midpointY = 0;

    let regionCount = -1;
    let unusedRegionId = [];

    $(document).ready(function() {
        // ELEMENTS
        // General controls
        this.statusBar = document.getElementById("statusBar");
        this.loadYaml = document.getElementById("loadYaml");
        this.loadImage = document.getElementById("loadImage");
        let title = document.getElementById("title");
        this.dbManageBtn = document.getElementById("manageDb");
        this.saveBtn = document.getElementById("save");
        let clear = document.getElementById("clear");

        // Program state variables
        title.value = "Please upload a .yaml file to begin";
        this.yamlUploaded = false;
        this.prevName = "";
        this.dbConnected = false;

        // Editor buttons
        let selectedShape = "Point";
        this.shapeTypeBtn = document.getElementById("shapeTypeBtn");
        this.addShapeBtn = document.getElementById("add");
        this.deleteShapeBtn = document.getElementById("delete");
        this.addEndpoint = document.getElementById("addEndpoint");
        this.deleteEndpoint = document.getElementById("deleteEndpoint");
        this.exitRegionEditor = document.getElementById("exitRegionEditor");

        // Svg and canvas
        this.stage = document.getElementById("stage");
        let canvas = document.getElementById('canvas');
        
        // Popup controls
        document.getElementById("manageDbPopupCloseBtn").addEventListener("click", closeManageDbPopup);
        document.getElementById("helpPopupCloseBtn").addEventListener("click", closeHelpPopup);
        document.getElementById("trackerPopupCloseBtn").addEventListener("click", closeTrackerPopup);
        this.disableDiv = document.getElementById("disableDiv");
        this.manageDbPopup = document.getElementById("manageDbPopup");
        this.mapListContainer = document.getElementById("mapList");
        this.savePopup = document.getElementById("savePopup");
        let rename = document.getElementById("rename");
        let saveAs = document.getElementById("saveAs");
        this.helpPopup = document.getElementById("helpPopup");
        this.helpPopupContent = document.getElementById("helpPopupContent");
        this.trackerPopup = document.getElementById("trackerPopup");
        this.trackerPopupContent = document.getElementById("trackerPopupContent");

        // ROS setup
        let websocketUrl = (function () {
            let hostname = window.location.hostname;
            let protocol = 'ws:';
            if (window.location.protocol === 'https:') {
                protocol = 'wss:';
            }
            return protocol + '//' + hostname + ':9090';
        })();
        this.ros = new ROSLIB.Ros({
            url: websocketUrl
        });
        this.ros.on('error', function (error) {
            self.statusBar.innerHTML = "Error connecting to ROS websocket server.";
            self.statusBar.style.color = RED;
        });
        this.ros.on('connection', function () {
            self.statusBar.innerHTML = "Connected to ROS!";
            self.statusBar.style.color = GREEN;
        });
        this.ros.on('close', function (error) {
            self.statusBar.innerHTML = "No connection to ROS.";
            self.statusBar.style.color = DARK_YELLOW;
        });
        // ROS topic
        let mapAnnotationTopic = new ROSLIB.Topic({
            ros: self.ros,
            name: "map_annotator/changes",
            messageType: "map_annotator_msgs/MapAnnotation"
        });
        let robotPositionTopic = new ROSLIB.Topic({
            ros: self.ros,
            name: "amcl_pose",
            messageType: "geometry_msgs/PoseWithCovarianceStamped"
        });
        robotPositionTopic.subscribe(displayRobotPosition);
        // ROS service
        this.getMapsService = new ROSLIB.Service({
            ros: self.ros,
            name: "map_annotator/get_maps",
            serviceType: "map_annotator_msgs/GetMaps"
        });
        this.deleteMapService = new ROSLIB.Service({
            ros: self.ros,
            name: "map_annotator/delete_map",
            serviceType: "map_annotator_msgs/DeleteMap"
        });
        this.hasPointService = new ROSLIB.Service({
            ros: self.ros,
            name: "map_annotator/has_point",
            serviceType: "map_annotator_msgs/HasPoint"
        });
        this.hasPoseService = new ROSLIB.Service({
            ros: self.ros,
            name: "map_annotator/has_pose",
            serviceType: "map_annotator_msgs/HasPose"
        });
        this.hasRegionService = new ROSLIB.Service({
            ros: self.ros,
            name: "map_annotator/has_region",
            serviceType: "map_annotator_msgs/HasRegion"
        });

        // Main parts
        this.editor = new Editor();
        this.editor.setup(this.stage, canvas);
        this.changeTracker = new ChangeTracker(mapAnnotationTopic, this.editor);
        this.selector = new Selector(this.stage, this.editor, this.changeTracker, 
                this.hasPointService, this.hasPoseService, this.hasRegionService,
                LINE_LENGTH, LABEL_PADDING);
        
        setEditorButtonStatus(true);

        // CONNECT/DISCONNECT TO DATABASE
        document.getElementById("connectDb").addEventListener('click', toggleDbConnection);

        // MANAGE DATABASE
        this.dbManageBtn.addEventListener('click', showManageDbPopup);

        // LOAD
        let form = document.createElement('form');
        form.style.display = 'none';
        document.body.appendChild(form);

        let input = document.createElement('input');
        input.type = 'file';
        input.addEventListener('change', function (event) {
            let file = input.files[0];
            let fileSuffix = file.name.split('.')[1];
            if (!self.yamlUploaded && (fileSuffix === 'yaml' || fileSuffix === 'yml')) {  // YAML
                let reader = new FileReader();
                reader.addEventListener('load', function (event) {
                    self.editor.clear();
                    self.changeTracker.reset();
                    self.stage.style.borderColor = WHITE;
                    // read YAML file
                    let contents = event.target.result;
                    // parse YAML file content to JSON
                    let contentsJSON = jsyaml.load(contents);
                    self.editor.setYAML(contentsJSON);
                    let fileNameArr = contentsJSON["image"].split(".")[0].split("/");
                    title.value = fileNameArr[fileNameArr.length - 1];
                    // enable image upload
                    self.yamlUploaded = true;
                    self.loadYaml.style.display = "none";
                    self.loadImage.style.display = "inline-block";
                    clear.disabled = false;
                    setEditorButtonStatus(true);
                    // enable clear button
                    clear.disabled = false;
                });
                reader.readAsText(file);
                form.reset();
            } else if (self.yamlUploaded) {
                if (fileSuffix === 'svg' || fileSuffix === 'pgm') {
                    title.value = file.name.split('.')[0];
                    self.prevName = title.value;
                    self.yamlUploaded = false;
                    setEditorButtonStatus(false);
                    self.loadYaml.style.display = "inline-block";
                    self.loadImage.style.display = "none";
                    self.stage.style.borderColor = DARK_GREY;
                    if (fileSuffix === 'svg') {  // file uploaded is SVG
                        let reader = new FileReader();
                        reader.addEventListener('load', function (event) {
                            // read SVG file
                            let contents = event.target.result;
                            self.editor.setSVG(new DOMParser().parseFromString(contents, 'image/svg+xml'));
                            // initialize the program state
                            initializeProgram();
                        }, false);
                        reader.readAsText(file);
                        form.reset();
                    } else {  // file uploaded is PGM
                        let reader = new FileReader();
                        reader.addEventListener('load', function (event) {
                            // read PGM file
                            let contents = event.target.result;
                            self.editor.setPGM(contents);
                            // initialize the program state
                            initializeProgram();
                        });
                        reader.readAsArrayBuffer(file);
                        form.reset();
                    }
                } else {
                    showHelpPopup("Please upload a PGM or SVG file!");
                }
            } else {
                showHelpPopup("Please upload a YAML file!");
            }
        });
        form.appendChild(input);

        document.getElementById("loadYaml").addEventListener('click', function () {
            input.click();
        });

        document.getElementById("loadImage").addEventListener('click', function () {
            input.click();
        });

        // SAVE
        let link = document.createElement('a');
        link.style.display = 'none';
        document.body.appendChild(link);

        this.saveBtn.addEventListener('click', function () {
            if (confirm("Are you sure you want to SAVE the changes?")) {
                // create a clone of the SVG node so we don't mess the original one
                let clone = self.editor.cloneStage();
                // remove the robot pose from the cloned SVG
                let robotPoseElement = clone.getElementById("robotPose");
                if (robotPoseElement) {
                    clone.removeChild(robotPoseElement);
                }
                // download SVG
                let svgBlob = new Blob([self.editor.toString(clone)], { type: 'image/svg+xml;charset=utf-8' });
                link.href = URL.createObjectURL(svgBlob);
                link.download = title.value + '.svg';
                link.click();

                if (self.dbConnected) {
                    // publish all the changes to ROS node to save everything to database
                    if (self.prevName === title.value) {
                        self.changeTracker.publishChanges("", title.value);
                        resetAfterSaving();
                    } else {
                        // ask the user if they want to rename the file or make a copy?
                        showSavePopup();
                    }
                } else {  // only download a local copy
                    resetAfterSaving();
                }
            }           
        });

        // Rename
        rename.addEventListener('click', function () {
            self.changeTracker.publishChanges(self.prevName, title.value);
            closeSavePopup();
            resetAfterSaving();
        });

        // Save as
        saveAs.addEventListener('click', function () {
            self.changeTracker.publishChanges(self.prevName, title.value, saveAs=true);
            closeSavePopup();
            resetAfterSaving();
        });

        // CLEAR
        clear.addEventListener('click', function () {
            if (self.yamlUploaded) {
                if (confirm("Are you sure you want to upload a new YAML file?")) {
                    // clear the uploaded yaml file, let the user to choose a new yaml file to upload
                    clearEditor();
                }
            } else if (confirm("Are you sure you want to DISCARD all the annotations?")) {
                clearAnnotations();
            }
        });

        // SHOW CHANGES
        document.getElementById("showChanges").addEventListener('click', function () {
            let changes = self.changeTracker.getChanges();
            showTrackerPopup(changes);
        });

        // EDITOR
        let shapeTypeDropdownList = document.querySelectorAll(".dropdown_content a");
        for (let i = 0; i < shapeTypeDropdownList.length; i++) {
            shapeTypeDropdownList[i].addEventListener("click", function() {
                selectedShape = markDropdownSelection(this);
            });
        }

        this.addShapeBtn.addEventListener('click', function () {
            // add the selected shape
            if (selectedShape === "Point") {
                addPoint();
            } else if (selectedShape === "Pose") {
                addPose();
            } else {
                addRegion();
            }
        });

        this.addShapeBtn.addEventListener('mouseover', function () {
            // show tooltip
            if (selectedShape === "Pose") {
                showTooltip("add", "Press \"SHIFT\" and click & drag to change orientation.");
            } else if (selectedShape === "Region"){
                showTooltip("add", "Click this button to add regions,</br>click on the region to edit it.");
            }
        });

        this.addShapeBtn.addEventListener('mouseout', function () {
            hideTooltip("add");
        });

        this.deleteShapeBtn.addEventListener('click', function () {
            if (this.innerHTML === "Delete") {
                // delete the selected shape
                this.innerHTML = "DONE";
                this.style.backgroundColor = HIGHLIGHT;
                self.addShapeBtn.disabled = true;
                self.shapeTypeBtn.disabled = true;
                if (selectedShape === "Point") {
                    self.selector.enterShapeDeleteMode("circle_annotation");
                } else if (selectedShape === "Pose") {
                    self.selector.enterShapeDeleteMode("pose_line_annotation");
                } else {  // Region
                    self.selector.enterShapeDeleteMode("region_annotation");
                }
            } else {
                // exit the delete mode
                this.innerHTML = "Delete";
                this.style.backgroundColor = WHITE;
                self.addShapeBtn.disabled = false;
                self.shapeTypeBtn.disabled = false;
                unusedRegionId = unusedRegionId.concat(self.selector.exitShapeDeleteMode());
            }
        });

        this.deleteShapeBtn.addEventListener('mouseover', function () {
            if (this.innerHTML === "Delete") {
                if (selectedShape === "Point") {
                    showTooltip("delete", "Click on the point you want to delete.");
                } else if (selectedShape === "Pose") {
                    showTooltip("delete", "Click on the pose you want to delete.");
                } else {  // Region
                    showTooltip("delete", "Click on the region you want to delete.");
                }
            }
        });

        this.deleteShapeBtn.addEventListener('mouseout', function () {
            hideTooltip("delete");
        });

        this.addEndpoint.addEventListener('click', function() {
            let selectedRegion = self.selector.getSelectedRegion();
            if (selectedRegion != null) {
                let stringOfPrevPoints = selectedRegion.getAttribute('points');
                let prevPoints = convertToList(stringOfPrevPoints);
                let newPoint = getNewEndpoint(prevPoints);
                selectedRegion.setAttribute('points', stringOfPrevPoints + " " + newPoint[0] + "," + newPoint[1]);
                // mark the new end point with a circle
                let regionId = getRegionId(selectedRegion);
                let circle = makeCircle(regionId, prevPoints.length, 'region_endpoint_annotation', newPoint[0], newPoint[1], DARK_YELLOW);
                let regionName = getLabelElement(selectedRegion).textContent;
                selectedRegion.parentElement.appendChild(circle);
                self.changeTracker.addRegionEndpoint(regionName, prevPoints.length, newPoint[0], newPoint[1]);
                // always update the list of points and translate
                let points = convertToList(selectedRegion.getAttribute('points'));
                let translate = getTranslate(selectedRegion.parentElement.getAttribute('transform'));
                self.changeTracker.applyRegionChange("translate", regionName, points, translate[0], translate[1]);
            } else {
                showHelpPopup("Please select a region first!");
            }
        });

        this.deleteEndpoint.addEventListener('click', function() {
            if (this.innerHTML === "Delete Endpoint") {
                // delete the selected endpoint
                this.innerHTML = "DONE";
                this.style.backgroundColor = HIGHLIGHT;
                self.addEndpoint.disabled = true;
                self.exitRegionEditor.disabled = true;
                self.selector.enterEndpointDeleteMode();
            } else {
                // exit the endpoint delete mode
                this.innerHTML = "Delete Endpoint";
                this.style.backgroundColor = WHITE;
                self.addEndpoint.disabled = false;
                self.exitRegionEditor.disabled = false;
                self.selector.exitEndpointDeleteMode();
            }
        });

        this.deleteEndpoint.addEventListener('mouseover', function() {
            if (this.innerHTML === "Delete Endpoint") {
                showTooltip("deleteEndpoint", "Press \"SHIFT\" and click on the endpoint</br>to delete it.");
            }
        });

        this.deleteEndpoint.addEventListener('mouseout', function () {
            hideTooltip("deleteEndpoint");
        });

        this.exitRegionEditor.addEventListener('click', function() {
            self.selector.exitRegionEditor();
        });

        document.getElementById("circleRadiusSlider").oninput = function() {
            // update circle radius
            circleRadius = this.value;
            self.editor.resizeCircles(circleRadius);
        }

        document.getElementById("strokeWidthSlider").oninput = function() {
            // update line width
            lineWidth = this.value;
            self.editor.resizeLines(lineWidth);
        }

        document.getElementById("fillBtn").addEventListener('click', function() {
            self.editor.fill();
        });

        document.getElementById("panSwitchBtn").addEventListener('click', function() {
            if (this.innerHTML === "Pan ON") {  // turn on pan
                self.editor.enablePan();
            } else {  // turn off pan
                self.editor.disablePan();
            }
        });
    });

    function addPoint() {
        let labelName = promptForName("point");
        if (labelName != "") {
            if (!self.changeTracker.hasPoint(labelName)) {
                if (self.dbConnected) {
                    // check if the point already exists in the database
                    let request = new ROSLIB.ServiceRequest({
                        map_name: self.prevName,
                        point_name: labelName
                    });
                    self.hasPointService.callService(request, function (result) {
                        if (!result.result) {
                            addPointWithName(labelName);
                        } else {
                            showHelpPopup("The point named \"" + labelName + "\" already exists!");
                        }
                    });
                } else {
                    addPointWithName(labelName);
                }
            } else {
                showHelpPopup("The point named \"" + labelName + "\" already exists!");
            }
        }
    }

    function addPose() {
        let labelName = promptForName("pose");
        if (labelName != "") {
            if (!self.changeTracker.hasPose(labelName)) {
                if (self.dbConnected) {
                    // check if the pose already exists in the database
                    let request = new ROSLIB.ServiceRequest({
                        map_name: self.prevName,
                        pose_name: labelName
                    });
                    self.hasPoseService.callService(request, function (result) {
                        if (!result.result) {
                            addPoseWithName(labelName);
                        } else {
                            showHelpPopup("The point named \"" + labelName + "\" already exists!");
                        }
                    });
                } else {
                    addPoseWithName(labelName);
                }
            } else {
                showHelpPopup("The pose named \"" + labelName + "\" already exists!");
            }
        }
    }

    function addRegion() {
        let labelName = promptForName("region");
        if (labelName != "") {
            if (!self.changeTracker.hasRegion(labelName)) {
                if (self.dbConnected) {
                    // check if the region already exists in the database
                    let request = new ROSLIB.ServiceRequest({
                        map_name: self.prevName,
                        region_name: labelName
                    });
                    self.hasRegionService.callService(request, function (result) {
                        if (!result.result) {
                            addRegionWithName(labelName);
                        } else {
                            showHelpPopup("The region named \"" + labelName + "\" already exists!");
                        }
                    });
                } else {
                    addRegionWithName(labelName);
                }
            } else {
                showHelpPopup("The region named \"" + labelName + "\" already exists!");
            }
        }
    }

    function clearAnnotations() {
        self.editor.clearAnnotations();
        self.changeTracker.reset();
    }

    function clearEditor() {
        setEditorButtonStatus(true);
        self.editor.clear();
        self.changeTracker.reset();
        title.value = "Please upload a .yaml file to begin";
        self.prevName = "";
        self.loadYaml.style.display = "inline-block";
        self.loadImage.style.display = "none";
        self.yamlUploaded = false;
        self.stage.style.borderColor = WHITE;
    }

    function displayRobotPosition(msg) {
        if (self.editor.isReadyToUse()) {
            let position = new ROSLIB.Vector3(msg.pose.pose.position);
            let orientation = new ROSLIB.Quaternion(msg.pose.pose.orientation);
            // convert X and Y from map coordinates to pixel coordinates
            let pixelCoordinate = self.editor.getPixelCoordinate(position.x, position.y);
            let x = pixelCoordinate[0];
            let y = pixelCoordinate[1];
            // calculate yaw
            let t = UNIT_VECTOR.clone();
            t.multiplyQuaternion(orientation);
            let theta = Math.atan2(UNIT_VECTOR.y, UNIT_VECTOR.x) - Math.atan2(t.y, t.x);

            // display the pose
            let robotPose = self.stage.getElementById("robotPose");
            let labelY = y + LINE_LENGTH + LABEL_PADDING;
            let poseX2 = x + LINE_LENGTH * Math.cos(theta);
            let poseY2 = y + LINE_LENGTH * Math.sin(theta);
            if (robotPose) {
                let poseLabel = robotPose.childNodes[0];
                poseLabel.setAttribute('x', x);
                poseLabel.setAttribute('y', labelY);
                let poseLine = robotPose.childNodes[1];
                poseLine.setAttribute('x1', x);
                poseLine.setAttribute('y1', y);
                poseLine.setAttribute('x2', poseX2);
                poseLine.setAttribute('y2', poseY2);
            } else {
                let poseGroup = document.createElementNS(NS, 'g');
                poseGroup.id = "robotPose";
                let label = makeLabel(x, labelY, GREEN, "Robot");
                poseGroup.appendChild(label);
                // arrow head
                let arrowhead = makeArrowhead(GREEN);
                let arrowmarker = makeArrowmarker("RobotPoseArrowMarker");
                arrowmarker.appendChild(arrowhead);
                self.editor.addElement(arrowmarker);
                // arrow line
                let arrowline = makeArrowline(x, y, poseX2, poseY2, "RobotPoseArrowMarker", GREEN);
                poseGroup.appendChild(arrowline);
                self.editor.addElement(poseGroup);
            }
        }
    }


    /////////////////// Helper functions ///////////////////

    function addPointWithName(labelName) {
        let pointGroup = document.createElementNS(NS, 'g');
        let label = makeLabel(midpointX, midpointY + LABEL_PADDING, RED, labelName);
        pointGroup.appendChild(label);
        let circle = makeCircle(-1, -1, 'circle_annotation', midpointX, midpointY, RED);
        pointGroup.appendChild(circle);
        self.editor.addElement(pointGroup);
        self.changeTracker.applyPointChange("save", labelName, midpointX, midpointY);
    }

    function addPoseWithName(labelName) {
        let poseGroup = document.createElementNS(NS, 'g');
        let label = makeLabel(midpointX, midpointY + LINE_LENGTH + LABEL_PADDING, BLUE, labelName);
        poseGroup.appendChild(label);
        // arrow head
        let arrowhead = makeArrowhead(BLUE);
        let arrowmarker = makeArrowmarker(labelName);
        arrowmarker.appendChild(arrowhead);
        self.editor.addElement(arrowmarker);
        // arrow line
        let arrowline = makeArrowline(midpointX, midpointY, midpointX + LINE_LENGTH, midpointY, labelName, BLUE);
        poseGroup.appendChild(arrowline);
        self.editor.addElement(poseGroup);
        self.changeTracker.applyPoseChange("save", labelName, midpointX, midpointY, 0);
    }

    function addRegionWithName(labelName) {
        let regionId;
        if (unusedRegionId.length > 0) {
            regionId = unusedRegionId.pop();
        } else {
            regionCount++;
            regionId = regionCount;
        }
        let regionGroup = document.createElementNS(NS, 'g');
        regionGroup.setAttribute("transform", "translate(0, 0)");
        // the region label position is relative to endpoint #0
        let label = makeLabel(midpointX, midpointY - LABEL_PADDING, DARK_YELLOW, labelName);
        regionGroup.appendChild(label);
        // add a triangle to start
        let basicRegion = document.createElementNS(NS, 'polygon');
        let points = [[midpointX, midpointY], [midpointX + INITIAL_REGION_SIZE, midpointY],
        [midpointX, midpointY + INITIAL_REGION_SIZE]];
        basicRegion.setAttribute('class', 'region_annotation');
        basicRegion.setAttribute('points', convertToString(points));
        basicRegion.style.fill = 'transparent';
        basicRegion.style.stroke = YELLOW;
        basicRegion.style.strokeWidth = lineWidth;
        regionGroup.appendChild(basicRegion);
        // mark end points with circles
        for (let i = 0; i < points.length; i++) {
            let point = points[i];
            let circle = makeCircle(regionId, i, 'region_endpoint_annotation', point[0], point[1], DARK_YELLOW);
            regionGroup.appendChild(circle);
        }
        self.editor.addElement(regionGroup);
        self.changeTracker.applyRegionChange("save", labelName, points);
    }
    
    function makeLabel(x, y, color, defaultText) {
        let label = document.createElementNS(NS, 'text');
        if (color != GREEN) {
            label.setAttribute('class', 'text_annotation');
        }
        label.setAttribute('x', x);
        label.setAttribute('y', y);
        label.setAttribute('font-size', LABEL_FONT_SIZE);
        label.style.stroke = color;
        label.style.fill = color;
        label.textContent = defaultText;
        return label;
    }

    function makeCircle(regionId, pointId, className, cx, cy, color) {
        let circle = document.createElementNS(NS, 'circle');
        if (regionId >= 0) {  // add an id number to the circle
            circle.setAttribute('id', regionId + "-" + pointId);
        }
        circle.setAttribute('class', className);
        circle.setAttribute('cx', cx);
        circle.setAttribute('cy', cy);
        circle.setAttribute('r', circleRadius);
        circle.style.stroke = color;
        circle.style.fill = color;
        return circle;
    }

    function makeArrowhead(color) {
        let arrowhead = document.createElementNS(NS, 'polygon');
        arrowhead.style.fill = color;
        arrowhead.setAttribute('points', "0 0,2 1,0 2");
        return arrowhead;
    }

    function makeArrowmarker(arrowMarkerId) {
        let arrowmarker = document.createElementNS(NS, 'marker');
        arrowmarker.setAttribute('id', 'arrowhead' + arrowMarkerId);
        arrowmarker.setAttribute('markerWidth', 2);
        arrowmarker.setAttribute('markerHeight', 2);
        arrowmarker.setAttribute('refX', 0);
        arrowmarker.setAttribute('refY', 1);
        arrowmarker.setAttribute('orient', "auto");
        return arrowmarker;
    }

    function makeArrowline(x1, y1, x2, y2, arrowMarkerId, color) {
        let arrowline = document.createElementNS(NS, 'line');
        if (color != GREEN) {
            arrowline.setAttribute('class', 'pose_line_annotation');
        } else {
            arrowline.setAttribute('class', 'robot_pose_line_annotation');
        }
        arrowline.setAttribute('x1', x1);
        arrowline.setAttribute('y1', y1);
        arrowline.setAttribute('x2', x2);
        arrowline.setAttribute('y2', y2);
        arrowline.setAttribute('marker-end', "url(#arrowhead" + arrowMarkerId + ")");
        arrowline.style.stroke = color;
        arrowline.style.strokeWidth = lineWidth;
        return arrowline;
    }

    function getNewEndpoint(prevPoints) {
        let minX = Infinity;
        let maxX = -Infinity;
        let minY = Infinity;
        let maxY = -Infinity;
        for (let i = 0; i < prevPoints.length; i++) {
            let point = prevPoints[i];
            minX = Math.min(minX, point[0]);
            maxX = Math.max(maxX, point[0]);
            minY = Math.min(minY, point[1]);
            maxY = Math.max(maxY, point[1]);
        }
        return [getRandomInteger(minX - 10, maxX - 10), getRandomInteger(minY - 10, maxY - 10)];
    }

    function initializeProgram() {
        // reset the change tracker
        self.changeTracker.reset();
        self.changeTracker.setMapName(title.value);
        // now we know the image size, calculate the mid coordinate
        midpointX = self.editor.getMidpointX();
        midpointY = self.editor.getMidpointY();
    }

    function setEditorButtonStatus(disabled) {
        let editorBtns = document.getElementsByClassName("editorBtn");
        for (let i = 0; i < editorBtns.length; i++) {
            editorBtns[i].disabled = disabled;
        }
    }

    function markDropdownSelection(selectedItem) {
        // mark the selection and return the selected item name
        selectedItem.parentElement.parentElement.querySelector(".dropbtn").innerHTML = selectedItem.innerHTML;
        return selectedItem.innerHTML;
    }

    function toggleDbConnection() {
        if (!self.dbConnected) {
            // show a confirmation box, because the editor will be cleared after the switch
            if (confirm("Are you sure you want to CONNECT to database? " +
                        "The svg editor will be CLEARED after the switch!")) {
                if (self.statusBar.innerHTML === "Connected to ROS!") {
                    // connect to database
                    clearEditor();
                    this.innerText = "Disconnect from Database";
                    this.style.backgroundColor = DARK_YELLOW;
                    self.dbManageBtn.style.display = "block";
                    self.saveBtn.innerText = "Save to DB & Download";
                    self.dbConnected = true;
                } else {
                    showHelpPopup("Cannot connect to database, please refresh the page.");
                }
            }
        } else {
            // disconnect from database
            this.innerText = "Connect to Database";
            this.style.backgroundColor = GREEN;
            self.dbManageBtn.style.display = "none";
            self.saveBtn.innerText = "Download Image";
            self.dbConnected = false;
        }
    }

    function showManageDbPopup() {
        self.manageDbPopup.style.display = "block";
        // disable everything in the background
        self.disableDiv.style.display = "block";
        // call the ROS service to fetch names of all the maps
        let request = new ROSLIB.ServiceRequest({});
        self.getMapsService.callService(request, function(result) {
            self.mapListContainer.innerHTML = "";
            let mapList = result.maps;
            for (let i = 0; i < mapList.length; i++) {
                let mapEntryDiv = document.createElement("div");
                let mapEntry = document.createElement("p");
                mapEntry.innerText = mapList[i];
                mapEntryDiv.appendChild(mapEntry);
                mapEntryDiv.appendChild(createDeleteMapBtn());
                self.mapListContainer.appendChild(mapEntryDiv);
            }
        });
    }

    function createDeleteMapBtn() {
        let deleteBtn = document.createElement("button");
        deleteBtn.innerText = "DELETE";
        deleteBtn.addEventListener("click", function() {
            let mapEntryDiv = this.parentElement;
            let mapEntryName = mapEntryDiv.firstChild.innerText;
            let deleteConfirm = confirm("Are you sure you want to DELETE the map named " + mapEntryName.toUpperCase() + "?");
            if (deleteConfirm == true) {
                // call the ROS service to delete the map
                let request = new ROSLIB.ServiceRequest({
                    name: mapEntryName
                });
                self.deleteMapService.callService(request, function(result) {
                    self.mapListContainer.removeChild(mapEntryDiv);
                });
            }
        })
        return deleteBtn;
    }

    function showSavePopup() {
        self.savePopup.style.display = "block";
        // disable everything in the background
        self.disableDiv.style.display = "block";
    }

    function resetAfterSaving() {
        self.prevName = title.value;
        self.changeTracker.reset();
    }

    function showTooltip(type, text) {
        document.getElementById(type + "Tooltip").style.visibility = "visible";
        document.getElementById(type + "Tooltip").innerHTML = text;
    }

    function hideTooltip(type) {
        document.getElementById(type + "Tooltip").style.visibility = "hidden";
    }

    function showHelpPopup(content) {
        showPopup(self.helpPopup, self.helpPopupContent, self.disableDiv, content);
    }

    function showTrackerPopup(content) {
        // display the pop up window
        self.trackerPopup.style.display = "block";
        self.trackerPopupContent.innerHTML = content;
        // disable everything in the background
        self.disableDiv.style.display = "block";
    }

    function closeManageDbPopup() {
        // close the pop up window
        self.manageDbPopup.style.display = "none";
        // enable everything in the background
        self.disableDiv.style.display = "none";
    }

    function closeSavePopup() {
        // close the pop up window
        self.savePopup.style.display = "none";
        // enable everything in the background
        self.disableDiv.style.display = "none";
    }

    function closeHelpPopup() {
        // close the pop up window
        self.helpPopup.style.display = "none";
        // enable everything in the background
        self.disableDiv.style.display = "none";
    }

    function closeTrackerPopup() {
        // close the pop up window
        self.trackerPopup.style.display = "none";
        // enable everything in the background
        self.disableDiv.style.display = "none";
    }
});