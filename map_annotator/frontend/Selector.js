import Point from './types_js/Point.js';
import {round, getTranslate, convertToDeg, getLabelElement, showPopup, convertToList, convertToString, promptForName, getRegionId} from './utils.js';
export default class Selector {
    constructor(stage, editor, changeTracker, hasPointService, hasPoseService, hasRegionService, lineLength, labelPadding) {
        this.REGION_HIGHLIGHT = '#ffe34c';

        let self = this;
        this.stage = stage;
        this.selected = null;
        this.inShapeDeleteMode = false;
        this.inEndpointDeleteMode = false;
        this.typeToDelete = '';
        this.editor = editor;
        this.changeTracker = changeTracker;
        this.hasPointService = hasPointService;
        this.hasPoseService = hasPoseService;
        this.hasRegionService = hasRegionService;

        this.disableDiv = document.getElementById('disableDiv');
        this.helpPopup = document.getElementById('helpPopup');
        this.helpPopupContent = document.getElementById('helpPopupContent');

        this.offset = {x: 0, y: 0};
        this.selectedRegion = null;
        this.deletedRegionIds = [];
        let angleOffset = 0;  // angle offset for pose orientation (in radians)

        this.prevHighlight = null;
        this.prevColor = '';

        function displayCircleInfo(x, y) {
            let mapCoordinate = self.editor.getMapCoordinate(x, y);
            document.getElementById('coordinateInfo').innerHTML =
                '<p>X: ' + round(mapCoordinate[0]) + '(map) ' + round(x) + '(px)</p>' +
                '<p>Y: ' + round(mapCoordinate[1]) + '(map) ' + round(y) + '(px)</p>';
        }

        function displayPoseInfo(x, y, theta) {
            let mapCoordinate = self.editor.getMapCoordinate(x, y);
            document.getElementById('coordinateInfo').innerHTML =
                '<p>X: ' + round(mapCoordinate[0]) + '(map) ' + round(x) + '(px)</p>' +
                '<p>Y: ' + round(mapCoordinate[1]) + '(map) ' + round(y) + '(px)</p>' +
                '<p>THETA: ' + round(-theta) + 'rad ' + round(convertToDeg(-theta)) + 'deg</p>';
        }

        function clearCoordinateInfo() {
            document.getElementById('coordinateInfo').innerHTML = '';
        }

        // HOVER
        stage.addEventListener('mouseover', function (event) {
            let target = event.target;
            let targetType = target.getAttribute('class');
            if (targetType === 'circle_annotation') {
                displayCircleInfo(target.getAttribute('cx'), target.getAttribute('cy'));
            } else if (targetType === 'pose_line_annotation' || targetType === 'robot_pose_line_annotation') {
                let x1 = target.getAttribute('x1');
                let y1 = target.getAttribute('y1');
                let x2 = target.getAttribute('x2');
                let y2 = target.getAttribute('y2');
                angleOffset = Math.atan2(y2 - y1, x2 - x1);
                displayPoseInfo(x1, y1, angleOffset);
            } else if (targetType === 'region_endpoint_annotation') {
                let translate = getTranslate(target.parentElement.getAttribute('transform'));
                let translatedX = parseFloat(target.getAttribute('cx')) + translate[0];
                let translatedY = parseFloat(target.getAttribute('cy')) + translate[1];
                displayCircleInfo(translatedX, translatedY);
            }
            self.updateSelection(target);
            // if (targetType && targetType !=="text_annotation" && targetType.endsWith("annotation")) {
            // 	target.style.cursor = "grab";
            // }
        });

        stage.addEventListener('mouseout', function () {
            clearCoordinateInfo();
            self.cancelPrevSelection();
        });

        // DRAG & DROP
        stage.addEventListener('mousedown', function (event) {
            if (!self.isPanEnabled()) {
                let target = event.target;
                let targetType = target.getAttribute('class');
                if (!target.isSameNode(stage) && targetType &&
                    !targetType.startsWith('svg-pan-zoom') && !self.isRobotPoseAnnotation(target)) {
                    self.offset.x = event.clientX;
                    self.offset.y = event.clientY;
                    if (targetType === 'pose_line_annotation' && self.selectedRegion === null) {
                        angleOffset = Math.atan2(target.getAttribute('y2') - target.getAttribute('y1'),
                            target.getAttribute('x2') - target.getAttribute('x1'));
                    } else if (targetType === 'region_annotation') {
                        target = self.makeRegionAnnotationSelection(target, event.clientX, event.clientY);
                    }
                    self.selected = target;
                }
            }
        });

        window.addEventListener('mousemove', function (event) {
            if (event.target.getAttribute('class') === 'region_annotation') {
                self.updateSelection(self.makeRegionAnnotationSelection(event.target, event.clientX, event.clientY));
            }
            if (!self.isPanEnabled() && self.selected) {
                let targetType = self.selected.getAttribute('class');
                let label = getLabelElement(self.selected);
                if (targetType === 'circle_annotation' && self.selectedRegion === null) {
                    let newPointOffset = self.calculateNewOffset(
                        parseFloat(self.selected.getAttribute('cx')),
                        parseFloat(self.selected.getAttribute('cy')),
                        event.clientX, event.clientY);
                    self.selected.setAttribute('cx', newPointOffset.x);
                    self.selected.setAttribute('cy', newPointOffset.y);
                    displayCircleInfo(newPointOffset.x, newPointOffset.y);
                    self.changeTracker.applyPointChange('save', label.textContent, newPointOffset.x, newPointOffset.y);
                    self.updateLabelPosition(label, event.clientX, event.clientY);
                } else if (targetType === 'pose_line_annotation' && self.selectedRegion === null) {
                    let x1 = parseFloat(self.selected.getAttribute('x1'));
                    let y1 = parseFloat(self.selected.getAttribute('y1'));
                    if (!event.shiftKey) {  // move pose
                        let newOffset = self.calculateNewOffset(x1, y1, event.clientX, event.clientY);
                        self.selected.setAttribute('x1', newOffset.x);
                        self.selected.setAttribute('y1', newOffset.y);
                        self.selected.setAttribute('x2', newOffset.x + lineLength * Math.cos(angleOffset));
                        self.selected.setAttribute('y2', newOffset.y + lineLength * Math.sin(angleOffset));
                        displayPoseInfo(newOffset.x, newOffset.y, angleOffset);
                        self.changeTracker.applyPoseChange('save', label.textContent, newOffset.x, newOffset.y, angleOffset);
                        self.updateLabelPosition(label, event.clientX, event.clientY);
                    } else {  // shift key pressed, change the arrow orientation
                        let x2 = parseFloat(self.selected.getAttribute('x2'));
                        let y2 = parseFloat(self.selected.getAttribute('y2'));
                        let newOffset = self.calculateNewOffset(x2, y2, event.clientX, event.clientY);
                        angleOffset = Math.atan2(newOffset.y - y1, newOffset.x - x1);
                        self.selected.setAttribute('x2', x1 + lineLength * Math.cos(angleOffset));
                        self.selected.setAttribute('y2', y1 + lineLength * Math.sin(angleOffset));
                        displayPoseInfo(x1, y1, angleOffset);
                        self.changeTracker.applyPoseChange('save', label.textContent, x1, y1, angleOffset);
                    }
                } else if (targetType === 'region_annotation') {
                    let regionGroup = self.selected.parentElement;
                    let referencePointElement = regionGroup.childNodes[2];
                    let translate = getTranslate(regionGroup.getAttribute('transform'));
                    let oldUntransformedX = parseFloat(referencePointElement.getAttribute('cx'));
                    let oldUntransformedY = parseFloat(referencePointElement.getAttribute('cy'));
                    let newOffset = self.calculateNewOffset(
                        oldUntransformedX + translate[0], oldUntransformedY + translate[1],
                        event.clientX, event.clientY);
                    let newTranslateX = newOffset.x - oldUntransformedX;
                    let newTranslateY = newOffset.y - oldUntransformedY;
                    regionGroup.setAttribute('transform', 'translate(' + newTranslateX + ',' + newTranslateY + ')');
                    // always update the list of points and translate
                    let currentRegion = regionGroup.childNodes[1];
                    let points = convertToList(currentRegion.getAttribute('points'));
                    self.changeTracker.applyRegionChange('translate', label.textContent, points, newTranslateX, newTranslateY);
                } else if (targetType === 'region_endpoint_annotation') {
                    // move the endpoint
                    let newOffset = self.calculateNewOffset(
                        parseFloat(self.selected.getAttribute('cx')),
                        parseFloat(self.selected.getAttribute('cy')),
                        event.clientX, event.clientY);
                    self.selected.setAttribute('cx', newOffset.x);
                    self.selected.setAttribute('cy', newOffset.y);
                    displayCircleInfo(newOffset.x, newOffset.y);
                    // adjust the line
                    let regionGroup = self.selected.parentElement;
                    let currentRegion = regionGroup.childNodes[1];
                    let points = convertToList(currentRegion.getAttribute('points'));
                    let selectedEndpointId = parseInt(self.selected.getAttribute('id').split('-')[1]);
                    points[selectedEndpointId] = [newOffset.x, newOffset.y];
                    currentRegion.setAttribute('points', convertToString(points));
                    // adjust the region label position so that it's relative to endpoint #0
                    if (selectedEndpointId === 0) {
                        self.updateLabelPosition(regionGroup.childNodes[0], event.clientX, event.clientY);
                    }
                    // track change
                    self.changeTracker.moveRegionEndpoint(label.textContent, selectedEndpointId, newOffset.x, newOffset.y);
                    // always update the translate
                    let translate = getTranslate(regionGroup.getAttribute('transform'));
                    self.changeTracker.applyRegionChange('translate', label.textContent, points, translate[0], translate[1]);
                } else if (targetType === 'text_annotation') {
                    // move label
                    let labelType = self.selected.parentElement.childNodes[1].getAttribute('class');
                    if (event.shiftKey && self.isValidLabelSelection(labelType)) {
                        self.updateLabelPosition(label, event.clientX, event.clientY);
                    }
                }
                self.updateSelection(self.selected);
                self.offset.x = event.clientX;
                self.offset.y = event.clientY;
            }
        });

        stage.addEventListener('mouseup', function () {
            self.selected = null;
            clearCoordinateInfo();
        });

        stage.addEventListener('click', function (event) {
            let target = event.target;
            let targetType = target.getAttribute('class');
            if (!target.isSameNode(stage) && targetType && !targetType.startsWith('svg-pan-zoom')) {
                if (targetType === 'region_annotation') {
                    target = self.makeRegionAnnotationSelection(target, event.clientX, event.clientY);
                    targetType = target.getAttribute('class');
                }
                if (self.inShapeDeleteMode && targetType === self.typeToDelete) {
                    // DELETE shape
                    let elementName = getLabelElement(target).textContent;
                    if (targetType === 'circle_annotation') {
                        self.changeTracker.applyPointChange('delete', elementName);
                        self.editor.deleteElementOfType('points', target.parentElement);
                    } else if (targetType === 'pose_line_annotation') {
                        self.changeTracker.applyPoseChange('delete', elementName);
                        self.editor.deleteElementOfType('poses', target.parentElement);
                    } else if (targetType === 'region_annotation') {
                        self.deletedRegionIds.push(getRegionId(target));
                        self.changeTracker.applyRegionChange('delete', elementName);
                        self.editor.deleteElementOfType('regions', target.parentElement);
                    }
                } else if (targetType === 'region_annotation' && event.shiftKey) {
                    // edit selected region
                    self.enterRegionEditor(target);
                } else if (self.inEndpointDeleteMode && event.shiftKey &&
                    targetType === 'region_endpoint_annotation') {
                    // DELETE endpoint
                    let group = target.parentElement;
                    let region = group.childNodes[1];
                    self.highlightRegion(region);
                    let points = convertToList(region.getAttribute('points'));
                    let regionName = getLabelElement(target).textContent;
                    if (points.length === 3) {  // delete the entire region
                        self.deletedRegionIds.push(getRegionId(group.childNodes[2]));
                        self.editor.deleteElementOfType('regions', group);
                        self.changeTracker.applyRegionChange('delete', regionName);
                        self.selectedRegion = null;
                        // exit region editing mode
                        self.resetAndExitRegionEditor();
                    } else {  // delete the point
                        self.editor.deleteElementOfGroup(group, target);
                        let selectedEndpointId = parseInt(target.getAttribute('id').split('-')[1]);
                        points.splice(selectedEndpointId, 1);
                        region.setAttribute('points', convertToString(points));
                        // update endpoint ids
                        let regionId = getRegionId(region);
                        for (let i = 2; i < group.childNodes.length; i++) {
                            let newId = i - 2;
                            group.childNodes[i].setAttribute('id', regionId + '-' + newId);
                        }
                        self.changeTracker.updateRegionEndpoints(regionName, points);
                        // always update the translate
                        let translate = getTranslate(group.getAttribute('transform'));
                        self.changeTracker.applyRegionChange('translate', regionName, points, translate[0], translate[1]);
                        // adjust the region label position so that it's relative to endpoint #0
                        let textLabel = group.childNodes[0];
                        textLabel.setAttribute('x', points[0][0]);
                        textLabel.setAttribute('y', points[0][1] - labelPadding);
                    }
                } else if (targetType === 'text_annotation' && !event.shiftKey) {
                    let labelType = target.parentElement.childNodes[1].getAttribute('class');
                    if (self.isValidLabelSelection(labelType)) {
                        let newLabel = promptForName(target.textContent);
                        if (newLabel !== '') {
                            if (labelType === 'circle_annotation' && !self.changeTracker.hasPoint(newLabel)) {
                                if (self.dbConnected()) {
                                    // check if the point already exists in the database
                                    let request = new ROSLIB.ServiceRequest({
                                        map_name: self.changeTracker.getMapName(),
                                        point_name: newLabel
                                    });
                                    self.hasPointService.callService(request, function (result) {
                                        if (!result.result) {
                                            self.changeTracker.applyPointChange('rename', target.textContent,
                                                undefined, undefined, newLabel);
                                            target.textContent = newLabel;
                                        } else {
                                            showPopup(self.helpPopup, self.helpPopupContent, self.disableDiv,
                                                'The name "' + newLabel + '" already exists!');
                                        }
                                    });
                                } else {
                                    self.changeTracker.applyPointChange('rename', target.textContent,
                                        undefined, undefined, newLabel);
                                    target.textContent = newLabel;
                                }
                            } else if (labelType === 'pose_line_annotation' && !self.changeTracker.hasPose(newLabel)) {
                                if (self.dbConnected()) {
                                    // check if the pose already exists in the database
                                    let request = new ROSLIB.ServiceRequest({
                                        map_name: self.changeTracker.getMapName(),
                                        pose_name: newLabel
                                    });
                                    self.hasPoseService.callService(request, function (result) {
                                        if (!result.result) {
                                            self.changeTracker.applyPoseChange('rename', target.textContent,
                                                undefined, undefined, undefined, newLabel);
                                            target.textContent = newLabel;
                                        } else {
                                            showPopup(self.helpPopup, self.helpPopupContent, self.disableDiv,
                                                'The name "' + newLabel + '" already exists!');
                                        }
                                    });
                                } else {
                                    self.changeTracker.applyPoseChange('rename', target.textContent,
                                        undefined, undefined, undefined, newLabel);
                                    target.textContent = newLabel;
                                }
                            } else if (labelType === 'region_annotation' && !self.changeTracker.hasRegion(newLabel)) {
                                if (self.dbConnected()) {
                                    // check if the region already exists in the database
                                    let request = new ROSLIB.ServiceRequest({
                                        map_name: self.changeTracker.getMapName(),
                                        region_name: newLabel
                                    });
                                    self.hasRegionService.callService(request, function (result) {
                                        if (!result.result) {
                                            self.changeTracker.applyRegionChange('rename', target.textContent,
                                                undefined, undefined, undefined, newLabel);
                                            target.textContent = newLabel;
                                        } else {
                                            showPopup(self.helpPopup, self.helpPopupContent, self.disableDiv,
                                                'The name "' + newLabel + '" already exists!');
                                        }
                                    });
                                } else {
                                    self.changeTracker.applyRegionChange('rename', target.textContent,
                                        undefined, undefined, undefined, newLabel);
                                    target.textContent = newLabel;
                                }
                            } else {
                                showPopup(self.helpPopup, self.helpPopupContent, self.disableDiv,
                                    'The name "' + newLabel + '" already exists!');
                            }
                        }
                    }
                }
                self.updateSelection(target);
            }
        });
    }

    updateSelection(element) {
        // remove previous highlight
        this.cancelPrevSelection();
        if (!element.isSameNode(this.stage) && element.getAttribute('id') !== 'background_img' &&
            (element.getAttribute('class') && !element.getAttribute('class').startsWith('svg-pan-zoom'))) {
            this.prevHighlight = element;
            this.prevColor = element.style.stroke;
            element.style.stroke = '#603e3e';
            if (this.isCircle(element)) {
                element.style.fill = '#603e3e';
            }
        }
    }

    cancelPrevSelection() {
        if (this.prevHighlight) {
            this.prevHighlight.style.stroke = this.prevColor;
            if (this.isCircle(this.prevHighlight)) {
                this.prevHighlight.style.fill = this.prevColor;
            }
        }
    }

    isCircle(element) {
        // return true if the element is a circle, false otherwise
        let type = element.getAttribute('class');
        return type && (type === 'circle_annotation' || type === 'region_endpoint_annotation');
    }

    isText(element) {
        // return true if the element is a text, false otherwise
        let type = element.getAttribute('class');
        return type && type === 'text_annotation';
    }

    dbConnected() {
        return window.document.getElementById('connectDb').innerText === 'Disconnect from Database';
    }

    isPanEnabled() {
        return this.editor && this.editor.isEditorPanEnabled();
    }

    enterShapeDeleteMode(typeToDelete) {
        this.inShapeDeleteMode = true;
        this.typeToDelete = typeToDelete;
    }

    exitShapeDeleteMode() {
        this.inShapeDeleteMode = false;
        this.typeToDelete = '';
        let result = [];
        if (this.deletedRegionIds.length > 0) {
            result = this.deletedRegionIds;
            this.deletedRegionIds = [];
        }
        return result;
    }

    enterRegionEditor(target) {
        window.document.getElementById('regionShapeBtns').style.visibility = 'visible';
        $(':button:not(.regionShapeBtn, .popupCloseBtn, .panZoomBtn)').prop('disabled', true);
        // highlight the selected region
        this.highlightRegion(target);
    }

    resetAndExitRegionEditor() {
        document.getElementById('deleteEndpoint').innerHTML = 'Delete Endpoint';
        document.getElementById('deleteEndpoint').style.backgroundColor = '#ffffff';
        document.getElementById('addEndpoint').disabled = false;
        document.getElementById('exitRegionEditor').disabled = false;
        this.exitEndpointDeleteMode();
        this.exitRegionEditor();
    }

    exitRegionEditor() {
        window.document.getElementById('regionShapeBtns').style.visibility = 'hidden';
        $(':button:not(.regionShapeBtn, .popupCloseBtn)').prop('disabled', false);
        // de-highlight the selected region
        if (this.selectedRegion !== null) {
            this.selectedRegion.style.fill = 'transparent';
            this.selectedRegion = null;
        }
    }

    enterEndpointDeleteMode() {
        this.inEndpointDeleteMode = true;
    }

    exitEndpointDeleteMode() {
        this.inEndpointDeleteMode = false;
    }

    getSelectedRegion() {
        return this.selectedRegion;
    }

    highlightRegion(target) {
        // de-highlight the previously selected region
        if (this.selectedRegion) {
            this.selectedRegion.style.fill = 'transparent';
        }
        // highlight the selected region
        this.selectedRegion = target;
        this.selectedRegion.style.fill = this.REGION_HIGHLIGHT;
        this.selectedRegion.style.fillOpacity = '0.5';
    }

    updateLabelPosition(label, clientX, clientY) {
        let newLabelOffset = this.calculateNewOffset(
            parseFloat(label.getAttribute('x')),
            parseFloat(label.getAttribute('y')),
            clientX, clientY);
        label.setAttribute('x', newLabelOffset.x);
        label.setAttribute('y', newLabelOffset.y);
    }

    calculateNewOffset(oldX, oldY, currentOffsetX, currentOffsetY) {
        let x = currentOffsetX - this.offset.x;
        let y = currentOffsetY - this.offset.y;
        let angle = Math.atan2(y, x);
        let unzoomedDistance = this.editor.getUnzoomedLength(Math.sqrt(Math.pow(y, 2) + Math.pow(x, 2)));
        let newOffsetX = oldX + unzoomedDistance * Math.cos(angle);
        let newOffsetY = oldY + unzoomedDistance * Math.sin(angle);
        return {x: newOffsetX, y: newOffsetY, angle: angle};
    }

    isRobotPoseAnnotation(target) {
        return target.parentElement.getAttribute('id') === 'robotPose';
    }

    isValidLabelSelection(labelType) {
        return labelType === 'region_annotation' || (labelType !== 'region_annotation' && this.selectedRegion === null);
    }

    makeRegionAnnotationSelection(region, clientX, clientY) {
        // in case the region label is within the region, determine whether the mouse is over the label or the region
        let label = region.parentElement.childNodes[0];
        let labelRect = label.getBoundingClientRect();
        let left = labelRect.left + window.pageXOffset;
        let top = labelRect.top + window.pageYOffset;
        if (left <= clientX && clientX <= left + labelRect.width && top <= clientY && clientY <= top + labelRect.height) {
            region.style.cursor = 'text';
            return label;
        }
        region.style.cursor = 'default';
        return region;
    }
}