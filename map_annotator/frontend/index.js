"use strict";

$(function() {
    const NS = 'http://www.w3.org/2000/svg';
    const HIGHLIGHT = "#ffc30b";
    const WHITE = "#ffffff";
    const RED = "#f10000";
    const BLUE = "#4c4cff";
    const YELLOW = "#ffd700";
    const DARK_YELLOW = "#998100";
    const GREEN = "#a7c44c";
    const LABEL_FONT_SIZE = "15px";

    const CIRCLE_RADIUS = 3.5;
    const LINE_WIDTH = 4;
    const LINE_LENGTH = 25;
    const INITIAL_REGION_SIZE = 50;
    const LABEL_PADDING = 25;

    var self = this;

    var midpointX = 0;
    var midpointY = 0;

    var regionCount = -1;
    var unusedRegionId = [];

    $(document).ready(function() {
        // ELEMENTS
        this.statusBar = document.getElementById("statusBar");
        var title = document.getElementById("title");

        var selectedShape = "Point";
        this.shapeTypeBtn = document.getElementById("shapeTypeBtn");
        this.addShapeBtn = document.getElementById("add");
        this.deleteShapeBtn = document.getElementById("delete");
        this.addEndpoint = document.getElementById("addEndpoint");
        this.deleteEndpoint = document.getElementById("deleteEndpoint");
        this.exitRegionEditor = document.getElementById("exitRegionEditor");

        var load = document.getElementById("load");
        var save = document.getElementById("save");
        var clear = document.getElementById("clear");

        var stage = document.getElementById("stage");
        
        this.editor = new Editor();
        this.selector = new Selector(stage, this.editor, LINE_LENGTH, LABEL_PADDING);
        setEditorButtonStatus(true);

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
        this.pointAnnotationTopic = new ROSLIB.Topic({
            ros: self.ros,
            name: "map_annotator/point",
            messageType: "map_annotator_msgs/PointAnnotation"
        });
        this.poseAnnotationTopic = new ROSLIB.Topic({
            ros: self.ros,
            name: "map_annotator/pose",
            messageType: "map_annotator_msgs/PoseAnnotation"
        });
        this.regionAnnotationTopic = new ROSLIB.Topic({
            ros: self.ros,
            name: "map_annotator/region",
            messageType: "map_annotator_msgs/RegionAnnotation"
        });

        // LOAD
        var form = document.createElement('form');
        form.style.display = 'none';
        document.body.appendChild(form);

        var input = document.createElement('input');
        input.type = 'file';
        input.addEventListener('change', function (event) {
            var file = input.files[0];
            if (file.name.split('.')[1] === 'svg') {
                title.value = file.name.split('.')[0];
                var reader = new FileReader();
                reader.addEventListener('load', function (event) {
                    // read SVG file
                    var contents = event.target.result;
                    self.editor.setSVG(stage, new DOMParser().parseFromString(contents, 'image/svg+xml'));
                    setEditorButtonStatus(false);
                    // now we know the image size, calculate the mid coordinate
                    midpointX = self.editor.getMidpointX();
                    midpointY = self.editor.getMidpointY();
                }, false);
                reader.readAsText(file);
                form.reset();
            } else {
                alert("Please upload an SVG file!");
            }
        });
        form.appendChild(input);

        load.addEventListener('click', function () {
            input.click();
        });

        // SAVE
        var link = document.createElement('a');
        link.style.display = 'none';
        document.body.appendChild(link);

        save.addEventListener('click', function () {
            var save = confirm("Are you sure you want to SAVE the changes?");
            if (save == true) {
                var blob = new Blob([self.editor.toString()], { type: 'text/plain' });
                link.href = URL.createObjectURL(blob);
                link.download = title.value + '.svg';
                link.click();
            }           
        });

        // CLEAR
        clear.addEventListener('click', function () {
            var clear = confirm("Are you sure you want to DISCARD the changes?");
            if (clear == true) {
                self.editor.clear();
                setEditorButtonStatus(true);
                title.value = "Please upload an SVG file";
            }
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

        this.deleteShapeBtn.addEventListener('click', function () {
            if (this.innerHTML === "Delete") {
                // delete the selected shape
                this.innerHTML = "DONE";
                this.style.backgroundColor = HIGHLIGHT;
                self.addShapeBtn.disabled = true;
                self.shapeTypeBtn.disabled = true;
                if (selectedShape === "Point") {
                    alert("Click on the point you want to delete.");
                    self.selector.enterShapeDeleteMode("circle_annotation");
                } else if (selectedShape === "Pose") {
                    alert("Click on the pose you want to delete.");
                    self.selector.enterShapeDeleteMode("pose_line_annotation");
                } else {  // Region
                    alert("Click on the region you want to delete.");
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

        this.addEndpoint.addEventListener('click', function() {
            var selectedRegion = self.selector.getSelectedRegion();
            var stringOfPrevPoints = selectedRegion.getAttribute('points');
            var prevPoints = convertToList(stringOfPrevPoints);
            var newPoint = getNewEndpoint(prevPoints);
            selectedRegion.setAttribute('points', stringOfPrevPoints + " " + newPoint[0] + "," + newPoint[1]);
            // mark the new end point with a circle
            var regionId = getRegionId(selectedRegion);
            var circle = makeCircle(regionId, prevPoints.length, 'region_endpoint_annotation', newPoint[0], newPoint[1], DARK_YELLOW);
            selectedRegion.parentElement.appendChild(circle);
        });

        this.deleteEndpoint.addEventListener('click', function() {
            if (this.innerHTML === "Delete Endpoint") {
                alert("Press \"SHIFT\" and click on the endpoint to delete it.");
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

        this.exitRegionEditor.addEventListener('click', function() {
            self.selector.exitRegionEditor();
        });
    });

    function addPoint() {
        var pointGroup = document.createElementNS(NS, 'g');
        var label = makeLabel(midpointX, midpointY + LABEL_PADDING, RED, 'Point');
        pointGroup.appendChild(label);
        var circle = makeCircle(-1, -1, 'circle_annotation', midpointX, midpointY, RED);
        pointGroup.appendChild(circle);
        self.editor.addElement(pointGroup);
        publishPointAnnotationMsg("save", "Point", midpointX, midpointY);
    }

    function addPose() {
        alert("Press \"SHIFT\" and click & drag to change orientation.");
        var poseGroup = document.createElementNS(NS, 'g');
        var label = makeLabel(midpointX, midpointY + LINE_LENGTH + LABEL_PADDING, BLUE, 'Pose');
        poseGroup.appendChild(label);
        // arrow head
        var arrowhead = document.createElementNS(NS, 'polygon');
        arrowhead.style.fill = BLUE;
        arrowhead.setAttribute('points', "0 0,3 1.5,0 3");
        var arrowmarker = document.createElementNS(NS, 'marker');
        arrowmarker.setAttribute('id', 'arrowhead');
        arrowmarker.setAttribute('markerWidth', 3);
        arrowmarker.setAttribute('markerHeight', 3);
        arrowmarker.setAttribute('refX', 0);
        arrowmarker.setAttribute('refY', 1.5);
        arrowmarker.setAttribute('orient', "auto");
        arrowmarker.appendChild(arrowhead);
        self.editor.addElement(arrowmarker);
        // arrow body
        var line = document.createElementNS(NS, 'line');
        line.setAttribute('class', 'pose_line_annotation');
        line.setAttribute('x1', midpointX);
        line.setAttribute('y1', midpointY);
        line.setAttribute('x2', midpointX + LINE_LENGTH);
        line.setAttribute('y2', midpointY);
        line.setAttribute('marker-end', "url(#arrowhead)");
        line.style.stroke = BLUE;
        line.style.strokeWidth = LINE_WIDTH;
        poseGroup.appendChild(line);
        self.editor.addElement(poseGroup);
    }

    function addRegion() {
        var regionId;
        if (unusedRegionId.length > 0) {
            regionId = unusedRegionId.pop();
        } else {
            regionCount++;
            regionId = regionCount;
        }
        alert("Click \"Add\" button to add regions, click on the region to edit it");
        var regionGroup = document.createElementNS(NS, 'g');
        regionGroup.setAttribute("transform", "translate(0, 0)");
        var label = makeLabel(midpointX, midpointY - LABEL_PADDING, DARK_YELLOW, 'Region');
        regionGroup.appendChild(label);
        // add a triangle to start
        var basicRegion = document.createElementNS(NS, 'polygon');
        var points = [[midpointX, midpointY], [midpointX + INITIAL_REGION_SIZE, midpointY], 
                      [midpointX, midpointY + INITIAL_REGION_SIZE]];
        basicRegion.setAttribute('class', 'region_annotation');
        basicRegion.setAttribute('points', convertToString(points));
        basicRegion.style.fill = 'transparent';
        basicRegion.style.stroke = YELLOW;
        basicRegion.style.strokeWidth = LINE_WIDTH;
        regionGroup.appendChild(basicRegion);
        // mark end points with circles
        for (var i = 0; i < points.length; i++) {
            var point = points[i];
            var circle = makeCircle(regionId, i, 'region_endpoint_annotation', point[0], point[1], DARK_YELLOW);
            regionGroup.appendChild(circle);
        }
        self.editor.addElement(regionGroup);
    }

    /////////////////// Helper functions ///////////////////
    function publishPointAnnotationMsg(command, name, x, y) {
        let msg = new ROSLIB.Message({
            command: command,
            name: name,
            x: x,
            y: y
        });
        self.pointAnnotationTopic.publish(msg);
    }

    function publishPoseAnnotationMsg(command, name, x, y, theta) {
        let msg = new ROSLIB.Message({
            command: command,
            name: name,
            x: x,
            y: y,
            theta: theta
        });
        self.poseAnnotationTopic.publish(msg);
    }

    function publishRegionAnnotationMsg(command, name, x, y) {
        let msg = new ROSLIB.Message({
            command: command,
            name: name,
            x: x,
            y: y
        });
        self.regionAnnotationTopic.publish(msg);
    }
    
    function makeLabel(x, y, color, defaultText) {
        var label = document.createElementNS(NS, 'text');
        label.setAttribute('class', 'text_annotation');
        label.setAttribute('x', x);
        label.setAttribute('y', y);
        label.setAttribute('font-size', LABEL_FONT_SIZE);
        label.style.stroke = color;
        label.style.fill = color;
        label.textContent = defaultText;
        return label;
    }

    function makeCircle(regionId, pointId, className, cx, cy, color) {
        var circle = document.createElementNS(NS, 'circle');
        if (regionId >= 0) {  // add an id number to the circle
            circle.setAttribute('id', regionId + "-" + pointId);
        }
        circle.setAttribute('class', className);
        circle.setAttribute('cx', cx);
        circle.setAttribute('cy', cy);
        circle.setAttribute('r', CIRCLE_RADIUS);
        circle.style.stroke = color;
        circle.style.fill = color;
        return circle;
    }

    function getNewEndpoint(prevPoints) {
        var minX = Infinity;
        var maxX = -Infinity;
        var minY = Infinity;
        var maxY = -Infinity;
        for (var i = 0; i < prevPoints.length; i++) {
            var point = prevPoints[i];
            minX = Math.min(minX, point[0]);
            maxX = Math.max(maxX, point[0]);
            minY = Math.min(minY, point[1]);
            maxY = Math.max(maxY, point[1]);
        }
        return [getRandomInteger(minX - 10, maxX - 10), getRandomInteger(minY - 10, maxY - 10)];
    }

    function setEditorButtonStatus(disabled) {
        var editorBtns = document.getElementsByClassName("editorBtn");
        for (var i = 0; i < editorBtns.length; i++) {
            editorBtns[i].disabled = disabled;
        }
    }

    function markDropdownSelection(selectedItem) {
        // mark the selection and return the selected item name
        selectedItem.parentElement.parentElement.querySelector(".dropbtn").innerHTML = selectedItem.innerHTML;
        return selectedItem.innerHTML;
    }
});