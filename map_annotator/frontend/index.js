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

    let self = this;

    let midpointX = 0;
    let midpointY = 0;

    let regionCount = -1;
    let unusedRegionId = [];

    $(document).ready(function() {
        // ELEMENTS
        this.statusBar = document.getElementById("statusBar");
        let title = document.getElementById("title");
        this.prevName = "";
        this.currentName = "";

        let selectedShape = "Point";
        this.shapeTypeBtn = document.getElementById("shapeTypeBtn");
        this.addShapeBtn = document.getElementById("add");
        this.deleteShapeBtn = document.getElementById("delete");
        this.addEndpoint = document.getElementById("addEndpoint");
        this.deleteEndpoint = document.getElementById("deleteEndpoint");
        this.exitRegionEditor = document.getElementById("exitRegionEditor");

        let stage = document.getElementById("stage");

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
            name: "map_annotator/robot_pose",
            messageType: "map_annotator_msgs/RobotPose"
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
        this.changeTracker = new ChangeTracker(mapAnnotationTopic);
        this.selector = new Selector(stage, this.editor, this.changeTracker, 
                this.hasPointService, this.hasPoseService, this.hasRegionService,
                LINE_LENGTH, LABEL_PADDING);
        
        setEditorButtonStatus(true);

        // MANAGE DATABASE
        document.getElementById("manageDb").addEventListener('click', showManageDbPopup);

        // LOAD
        let form = document.createElement('form');
        form.style.display = 'none';
        document.body.appendChild(form);

        let input = document.createElement('input');
        input.type = 'file';
        input.addEventListener('change', function (event) {
            let file = input.files[0];
            if (file.name.split('.')[1] === 'svg') {
                title.value = file.name.split('.')[0];
                self.prevName = title.value;
                let reader = new FileReader();
                reader.addEventListener('load', function (event) {
                    // read SVG file
                    let contents = event.target.result;
                    self.editor.setSVG(stage, new DOMParser().parseFromString(contents, 'image/svg+xml'));
                    setEditorButtonStatus(false);
                    self.changeTracker.setMapName(title.value);
                    self.changeTracker.reset();
                    // now we know the image size, calculate the mid coordinate
                    midpointX = self.editor.getMidpointX();
                    midpointY = self.editor.getMidpointY();
                }, false);
                reader.readAsText(file);
                form.reset();
            } else {
                showHelpPopup("Please upload an SVG file!");
            }
        });
        form.appendChild(input);

        document.getElementById("load").addEventListener('click', function () {
            input.click();
        });

        // SAVE
        let link = document.createElement('a');
        link.style.display = 'none';
        document.body.appendChild(link);

        document.getElementById("save").addEventListener('click', function () {
            let save = confirm("Are you sure you want to SAVE the changes?");
            if (save == true) {
                let blob = new Blob([self.editor.toString()], { type: 'text/plain' });
                link.href = URL.createObjectURL(blob);
                self.currentName = title.value;
                link.download = self.currentName + '.svg';
                link.click();
                // publish all the changes to ROS node to save everything to database
                if (self.prevName === self.currentName) {
                    self.changeTracker.publishChanges("", self.currentName);
                } else {
                    // ask the user if they want to rename the file or make a copy?
                    showSavePopup();
                }
                // reset the change traker
                self.prevName = self.currentName;
                self.changeTracker.reset();
            }           
        });

        // Rename
        rename.addEventListener('click', function () {
            self.changeTracker.publishChanges(self.prevName, self.currentName);
            closeSavePopup();
        });

        // Save as
        saveAs.addEventListener('click', function () {
            self.changeTracker.publishChanges("", self.currentName);
            closeSavePopup();
        });

        // CLEAR
        document.getElementById("clear").addEventListener('click', function () {
            let clear = confirm("Are you sure you want to DISCARD the changes?");
            if (clear == true) {
                self.editor.clear();
                setEditorButtonStatus(true);
                title.value = "Please upload an SVG file";
                prevName = "";
                self.changeTracker.reset();
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

        this.deleteShapeBtn.addEventListener('click', function () {
            if (this.innerHTML === "Delete") {
                // delete the selected shape
                this.innerHTML = "DONE";
                this.style.backgroundColor = HIGHLIGHT;
                self.addShapeBtn.disabled = true;
                self.shapeTypeBtn.disabled = true;
                if (selectedShape === "Point") {
                    showHelpPopup("Click on the point you want to delete.");
                    self.selector.enterShapeDeleteMode("circle_annotation");
                } else if (selectedShape === "Pose") {
                    showHelpPopup("Click on the pose you want to delete.");
                    self.selector.enterShapeDeleteMode("pose_line_annotation");
                } else {  // Region
                    showHelpPopup("Click on the region you want to delete.");
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
                showHelpPopup("Press \"SHIFT\" and click on the endpoint to delete it.");
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
        let labelName = promptForName("point");
        if (labelName != "") {
            if (!self.changeTracker.hasPoint(labelName)) {
                // check if the point already exists in the database
                let request = new ROSLIB.ServiceRequest({
                    map_name: self.prevName,
                    point_name: labelName
                });
                self.hasPointService.callService(request, function (result) {
                    if (!result.result) {
                        let pointGroup = document.createElementNS(NS, 'g');
                        let label = makeLabel(midpointX, midpointY + LABEL_PADDING, RED, labelName);
                        pointGroup.appendChild(label);
                        let circle = makeCircle(-1, -1, 'circle_annotation', midpointX, midpointY, RED);
                        pointGroup.appendChild(circle);
                        self.editor.addElement(pointGroup);
                        self.changeTracker.applyPointChange("save", labelName, midpointX, midpointY);
                    } else {
                        showHelpPopup("The point named \"" + labelName + "\" already exists!");
                    }
                });
            } else {
                showHelpPopup("The point named \"" + labelName + "\" already exists!");
            }
        }
    }

    function addPose() {
        let labelName = promptForName("pose");
        if (labelName != "") {
            if (!self.changeTracker.hasPose(labelName)) {
                // check if the pose already exists in the database
                let request = new ROSLIB.ServiceRequest({
                    map_name: self.prevName,
                    pose_name: labelName
                });
                self.hasPoseService.callService(request, function (result) {
                    if (!result.result) {
                        showHelpPopup("Press \"SHIFT\" and click & drag to change orientation.");
                        let poseGroup = document.createElementNS(NS, 'g');
                        let label = makeLabel(midpointX, midpointY + LINE_LENGTH + LABEL_PADDING, BLUE, labelName);
                        poseGroup.appendChild(label);
                        // arrow head
                        let arrowhead = makeArrowhead();
                        let arrowmarker = makeArrowmarker();
                        arrowmarker.appendChild(arrowhead);
                        self.editor.addElement(arrowmarker);
                        // arrow line
                        let arrowline = makeArrowline();
                        poseGroup.appendChild(arrowline);
                        self.editor.addElement(poseGroup);
                        self.changeTracker.applyPoseChange("save", labelName, midpointX, midpointY, 0);
                    } else {
                        showHelpPopup("The point named \"" + labelName + "\" already exists!");
                    }
                });
            } else {
                showHelpPopup("The pose named \"" + labelName + "\" already exists!");
            }
        }
    }

    function addRegion() {
        let labelName = promptForName("region");
        if (labelName != "") {
            if (!self.changeTracker.hasRegion(labelName)) {
                // check if the region already exists in the database
                let request = new ROSLIB.ServiceRequest({
                    map_name: self.prevName,
                    region_name: labelName
                });
                self.hasRegionService.callService(request, function (result) {
                    if (!result.result) {
                        showHelpPopup("Click \"Add\" button to add regions, click on the region to edit it");
                        let regionId;
                        if (unusedRegionId.length > 0) {
                            regionId = unusedRegionId.pop();
                        } else {
                            regionCount++;
                            regionId = regionCount;
                        }
                        let regionGroup = document.createElementNS(NS, 'g');
                        regionGroup.setAttribute("transform", "translate(0, 0)");
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
                        basicRegion.style.strokeWidth = LINE_WIDTH;
                        regionGroup.appendChild(basicRegion);
                        // mark end points with circles
                        for (let i = 0; i < points.length; i++) {
                            let point = points[i];
                            let circle = makeCircle(regionId, i, 'region_endpoint_annotation', point[0], point[1], DARK_YELLOW);
                            regionGroup.appendChild(circle);
                        }
                        self.editor.addElement(regionGroup);
                        self.changeTracker.applyRegionChange("save", labelName, points);
                    } else {
                        showHelpPopup("The region named \"" + labelName + "\" already exists!");
                    }
                });
            } else {
                showHelpPopup("The region named \"" + labelName + "\" already exists!");
            }
        }
    }

    function displayRobotPosition(msg) {
        let theta = -msg.theta;
        console.log("X: " + msg.x);
        console.log("Y: " + msg.y);
        console.log("Theta: " + theta);
        // TODO: convert X and Y from map coordinates to pixel coordinates

        // TODO: display a pose
    }


    /////////////////// Helper functions ///////////////////
    
    function makeLabel(x, y, color, defaultText) {
        let label = document.createElementNS(NS, 'text');
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
        let circle = document.createElementNS(NS, 'circle');
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

    function makeArrowhead() {
        let arrowhead = document.createElementNS(NS, 'polygon');
        arrowhead.style.fill = BLUE;
        arrowhead.setAttribute('points', "0 0,3 1.5,0 3");
        return arrowhead;
    }

    function makeArrowmarker() {
        let arrowmarker = document.createElementNS(NS, 'marker');
        arrowmarker.setAttribute('id', 'arrowhead');
        arrowmarker.setAttribute('markerWidth', 3);
        arrowmarker.setAttribute('markerHeight', 3);
        arrowmarker.setAttribute('refX', 0);
        arrowmarker.setAttribute('refY', 1.5);
        arrowmarker.setAttribute('orient', "auto");
        return arrowmarker;
    }

    function makeArrowline() {
        let arrowline = document.createElementNS(NS, 'line');
        arrowline.setAttribute('class', 'pose_line_annotation');
        arrowline.setAttribute('x1', midpointX);
        arrowline.setAttribute('y1', midpointY);
        arrowline.setAttribute('x2', midpointX + LINE_LENGTH);
        arrowline.setAttribute('y2', midpointY);
        arrowline.setAttribute('marker-end', "url(#arrowhead)");
        arrowline.style.stroke = BLUE;
        arrowline.style.strokeWidth = LINE_WIDTH;
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

    function showManageDbPopup() {
        self.manageDbPopup.style.display = "block";
        // disable everything in the background
        self.disableDiv.style.display = "block";
        // call the ROS service to fetch names of all the maps
        let request = new ROSLIB.ServiceRequest({});
        self.getMapsService.callService(request, function(result) {
            console.log(result);
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