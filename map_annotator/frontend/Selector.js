class Selector {
	constructor(stage, editor, changeTracker, hasPointService, hasPoseService, hasRegionService, lineLength, labelPadding) {
		this.REGION_HIGHLIGHT = "#ffe34c";

		let self = this;

		this.selected = null;
		this.inShapeDeleteMode = false;
		this.inEndpointDeleteMode = false;
		this.typeToDelete = "";
		this.editor = editor;
		this.changeTracker = changeTracker;
		this.hasPointService = hasPointService;
		this.hasPoseService = hasPoseService;
		this.hasRegionService = hasRegionService;

		this.disableDiv = document.getElementById("disableDiv");
		this.helpPopup = document.getElementById("helpPopup");
		this.helpPopupContent = document.getElementById("helpPopupContent");

		let selection = document.createElement('span');
		selection.id = "selection";
		selection.style.position = 'absolute';
		selection.style.display = 'none';
		selection.style.outline = 'solid 2px #99f';
		selection.style.pointerEvents = 'none';
		document.body.appendChild(selection);

		this.offset = { x: 0, y: 0 };
		this.selectedRegion = null;
		this.deletedRegionIds = [];
		let angleOffset = 0;  // angle offset for pose orientation (in radians)
		let regionReferencePoint = { x: 0, y: 0 };
		let regionOffset = { x: 0, y: 0 };

		function updateSelection(element) {
			if (element.isSameNode(stage) || element.getAttribute('id') === "background_img" || 
				(element.getAttribute('class') && element.getAttribute('class').startsWith("svg-pan-zoom-control"))) {
				// remove previous highlight
				selection.style.display = 'none';
				return;
			}
			// highlight selection
			let rect = element.getBoundingClientRect();
			selection.style.left = (rect.left + window.pageXOffset) + 'px';
			selection.style.top = (rect.top + window.pageYOffset) + 'px';
			selection.style.width = rect.width + 'px';
			selection.style.height = rect.height + 'px';
			selection.style.display = 'block';
		}

		function displayCircleInfo(x, y) {
			let mapCoordinate = self.editor.getMapCoordinate(x, y);
			document.getElementById("coordinateInfo").innerHTML = 
				"<p>X: " + round(mapCoordinate[0]) + "(map) " + round(x) + "(px)</p>" +
				"<p>Y: " + round(mapCoordinate[1]) + "(map) " + round(y) + "(px)</p>";
		}

		function displayPoseInfo(x, y, theta) {
			let mapCoordinate = self.editor.getMapCoordinate(x, y);
			document.getElementById("coordinateInfo").innerHTML = 
				"<p>X: " + round(mapCoordinate[0]) + "(map) " + round(x) + "(px)</p>" +
				"<p>Y: " + round(mapCoordinate[1]) + "(map) " + round(y) + "(px)</p>" +
				"<p>THETA: " + round(-theta) + "rad " + round(convertToDeg(-theta)) + "deg</p>";
		}

		function clearCoordinateInfo() {
			document.getElementById("coordinateInfo").innerHTML = "";
		}

		// HOVER
		stage.addEventListener('mouseover', function (event) {
			let target = event.target;
			updateSelection(target);
			let targetType = target.getAttribute('class');
			if (targetType === 'circle_annotation') {
				displayCircleInfo(target.getAttribute('cx'), target.getAttribute('cy'));
			} else if (targetType === 'pose_line_annotation' || targetType === 'robot_pose_line_annotation') {
				let x1 = target.getAttribute('x1');
				let y1 = target.getAttribute('y1');
				let x2 = target.getAttribute('x2');
				let y2 = target.getAttribute('y2');
				let angle = Math.atan2(y2 - y1, x2 - x1);
				displayPoseInfo(x1, y1, angle);
			} else if (targetType === 'region_endpoint_annotation') {
				let translate = getTranslate(target.parentElement.getAttribute('transform'));
				let translatedX = parseFloat(target.getAttribute('cx')) + translate[0];
				let translatedY = parseFloat(target.getAttribute('cy')) + translate[1];
				displayCircleInfo(translatedX, translatedY);
			}
		});

		stage.addEventListener('mouseout', function () {
			clearCoordinateInfo();
		});

		// DRAG & DROP
		stage.addEventListener('mousedown', function (event) {
			let target = event.target;
			let targetType = target.getAttribute('class');
			if (!target.isSameNode(stage) && targetType && !targetType.startsWith("svg-pan-zoom-control")) {
				self.offset.x = event.offsetX;
				self.offset.y = event.offsetY;
				if (targetType === 'pose_line_annotation' && self.selectedRegion == null) {
					angleOffset = Math.atan2(target.getAttribute('y2') - target.getAttribute('y1'), 
											 target.getAttribute('x2') - target.getAttribute('x1'));
				} else if (targetType === 'region_annotation') {
					// let referenceGroup = target.parentElement;
					// let referencePointElement = referenceGroup.childNodes[2];
					// let translate = getTranslate(referenceGroup.getAttribute('transform'));
					// regionReferencePoint.x = parseFloat(referencePointElement.getAttribute('cx'));
					// regionReferencePoint.y = parseFloat(referencePointElement.getAttribute('cy'));
					// offset.x = regionReferencePoint.x + translate[0] - event.offsetX;
					// offset.y = regionReferencePoint.y + translate[1] - event.offsetY;
					// let referenceGroup = target.parentElement;
					// let referencePointElement = referenceGroup.childNodes[2];
					// let translate = getTranslate(referenceGroup.getAttribute('transform'));
					// regionReferencePoint.x = parseFloat(referencePointElement.getAttribute('cx'));
					// regionReferencePoint.y = parseFloat(referencePointElement.getAttribute('cy'));
					// regionOffset.x = regionReferencePoint.x + translate[0] - event.offsetX;
					// regionOffset.y = regionReferencePoint.y + translate[1] - event.offsetY;
					// console.log(regionOffset);
				}
				self.selected = target;
			}
		});

		window.addEventListener('mousemove', function (event) {
			if (self.selected) {
				let targetType = self.selected.getAttribute('class');
				let label = getLabelElement(self.selected);
				if (targetType === 'circle_annotation' && self.selectedRegion == null) {
					let newOffset = self.calculateNewOffset(
							parseFloat(self.selected.getAttribute('cx')), 
							parseFloat(self.selected.getAttribute('cy')), 
							event.offsetX, event.offsetY);
					self.selected.setAttribute('cx', newOffset.x);
					self.selected.setAttribute('cy', newOffset.y);
					displayCircleInfo(newOffset.x, newOffset.y);
					label.setAttribute('x', newOffset.x);
					label.setAttribute('y', newOffset.y + labelPadding);
					self.changeTracker.applyPointChange("save", label.textContent, newOffset.x, newOffset.y);
				} else if (targetType === 'pose_line_annotation' && self.selectedRegion == null) {
					let x1 = parseFloat(self.selected.getAttribute('x1'));
					let y1 = parseFloat(self.selected.getAttribute('y1'));
					let newOffset = self.calculateNewOffset(x1, y1, event.offsetX, event.offsetY);
					if (!event.shiftKey) {  // move pose
						self.selected.setAttribute('x1', newOffset.x);
						self.selected.setAttribute('y1', newOffset.y);
						label.setAttribute('x', newOffset.x);
						label.setAttribute('y', newOffset.y + lineLength + labelPadding);
						self.selected.setAttribute('x2', newOffset.x + lineLength * Math.cos(angleOffset));
						self.selected.setAttribute('y2', newOffset.y + lineLength * Math.sin(angleOffset));
						displayPoseInfo(newOffset.x, newOffset.y, angleOffset);
						self.changeTracker.applyPoseChange("save", label.textContent, newOffset.x, newOffset.y, angleOffset);
					} else {  // shift key pressed, change the arrow orientation

						angleOffset = Math.atan2(event.offsetY - y1, event.offsetX - x1);
						self.selected.setAttribute('x2', x1 + lineLength * Math.cos(angleOffset));
						self.selected.setAttribute('y2', y1 + lineLength * Math.sin(angleOffset));
						displayPoseInfo(x1, y1, angleOffset);
						self.changeTracker.applyPoseChange("save", label.textContent, x1, y1, angleOffset);
					}
				} else if (targetType === 'region_annotation') {
					let regionGroup = self.selected.parentElement;
					let referencePointElement = regionGroup.childNodes[2];
					let translate = getTranslate(regionGroup.getAttribute('transform'));
					// regionReferencePoint.x = parseFloat(referencePointElement.getAttribute('cx'));
					// regionReferencePoint.y = parseFloat(referencePointElement.getAttribute('cy'));
					
					let oldX = parseFloat(referencePointElement.getAttribute('cx')) + translate[0];
					let oldY = parseFloat(referencePointElement.getAttribute('cy')) + translate[1];

					let newOffset = self.calculateNewOffset(oldX, oldY, event.offsetX, event.offsetY);

					let newTranslateX = newOffset.x - parseFloat(referencePointElement.getAttribute('cx'));
					let newTranslateY = newOffset.y - parseFloat(referencePointElement.getAttribute('cy'));

					console.log(newTranslateX + ", " + newTranslateY);
					//

					regionGroup.setAttribute('transform', 'translate(' + newTranslateX + ',' + newTranslateY + ')');
					// always update the list of points and translate
					let currentRegion = regionGroup.childNodes[1];
					let points = convertToList(currentRegion.getAttribute('points'));
					self.changeTracker.applyRegionChange("translate", label.textContent, points, newTranslateX, newTranslateY);
				} else if (targetType === 'region_endpoint_annotation') {
					// move the endpoint
					let newOffset = self.calculateNewOffset(
						parseFloat(self.selected.getAttribute('cx')), 
						parseFloat(self.selected.getAttribute('cy')), 
						event.offsetX, event.offsetY);
					self.selected.setAttribute('cx', newOffset.x);
					self.selected.setAttribute('cy', newOffset.y);
					displayCircleInfo(newOffset.x, newOffset.y);
					// adjust the line
					let regionGroup = self.selected.parentElement;
					let currentRegion = regionGroup.childNodes[1];
					let points = convertToList(currentRegion.getAttribute('points'));
					let selectedEndpointId = parseInt(self.selected.getAttribute('id').split("-")[1]);
					points[selectedEndpointId] = [newOffset.x, newOffset.y];
					currentRegion.setAttribute('points', convertToString(points));
					// track change
					self.changeTracker.moveRegionEndpoint(label.textContent, selectedEndpointId, newOffset.x, newOffset.y);
					// always update the translate
					let translate = getTranslate(regionGroup.getAttribute('transform'));
					self.changeTracker.applyRegionChange("translate", label.textContent, points, translate[0], translate[1]);
				}
				updateSelection(self.selected);
				self.offset.x = event.offsetX;
				self.offset.y = event.offsetY;
			}
		});

		stage.addEventListener('mouseup', function () {
			self.selected = null;
			clearCoordinateInfo();
		});

		stage.addEventListener('click', function (event) {
			let target = event.target;
			let targetType = target.getAttribute('class');
			if (!target.isSameNode(stage) && targetType && !targetType.startsWith("svg-pan-zoom-control")) {
				if (self.inShapeDeleteMode && targetType === self.typeToDelete) {
					// DELETE shape
					let elementName = getLabelElement(target).textContent;
					if (targetType === 'circle_annotation') {
						self.changeTracker.applyPointChange("delete", elementName);
					} else if (targetType === 'pose_line_annotation') {
						self.changeTracker.applyPoseChange("delete", elementName);
					} else if (targetType === 'region_annotation') {
						self.deletedRegionIds.push(getRegionId(target));
						self.changeTracker.applyRegionChange("delete", elementName);
					}
					self.editor.deleteElement(target.parentElement);
				} else if (targetType === 'region_annotation') {
					// edit selected region
					self.enterRegionEditor(target);
				} else if (self.inEndpointDeleteMode && event.shiftKey === true && 
						   targetType === 'region_endpoint_annotation') {
					// DELETE endpoint
					let group = target.parentElement;
					let region = group.childNodes[1];
					self.highlightRegion(region);
					let points = convertToList(region.getAttribute('points'));
					let regionName = getLabelElement(target).textContent;
					if (points.length === 3) {  // delete the entire region
						self.deletedRegionIds.push(getRegionId(group.childNodes[2]));
						self.editor.deleteElement(group);
						self.changeTracker.applyRegionChange("delete", regionName);
						self.selectedRegion = null;
					} else {  // delete the point
						self.editor.deleteElementOfGroup(group, target);
						let selectedEndpointId = parseInt(target.getAttribute('id').split("-")[1]);
						points.splice(selectedEndpointId, 1);
						region.setAttribute('points', convertToString(points));
						// update endpoint ids
						let regionId = getRegionId(region);
						for (let i = 2; i < group.childNodes.length; i++) {
							let newId = i - 2;
							group.childNodes[i].setAttribute('id', regionId + "-" + newId);
						}
						self.changeTracker.updateRegionEndpoints(regionName, points);
						// always update the translate
						let translate = getTranslate(group.getAttribute('transform'));
						self.changeTracker.applyRegionChange("translate", regionName, points, translate[0], translate[1]);
					}
				} else if (targetType === 'text_annotation') {
					let labelType = target.parentElement.childNodes[1].getAttribute('class');
					if (labelType === "region_annotation" || 
						(labelType != "region_annotation" && self.selectedRegion == null)) {
						let newLabel = promptForName(target.textContent);
						if (newLabel != "") {
							if (labelType === "circle_annotation" && !self.changeTracker.hasPoint(newLabel)) {
								if (self.dbConnected()) {
									// check if the point already exists in the database
									let request = new ROSLIB.ServiceRequest({
										map_name: self.changeTracker.getMapName(),
										point_name: newLabel
									});
									self.hasPointService.callService(request, function (result) {
										if (!result.result) {
											self.changeTracker.applyPointChange("rename", target.textContent, 
													undefined, undefined, newLabel);
											target.textContent = newLabel;
										} else {
											showPopup(self.helpPopup, self.helpPopupContent, self.disableDiv,
												"The name \"" + newLabel + "\" already exists!");
										}
									});
								} else {
									self.changeTracker.applyPointChange("rename", target.textContent, 
													undefined, undefined, newLabel);
									target.textContent = newLabel;
								}
							} else if (labelType === "pose_line_annotation" && !self.changeTracker.hasPose(newLabel)) {
								if (self.dbConnected()) {
									// check if the pose already exists in the database
									let request = new ROSLIB.ServiceRequest({
										map_name: self.changeTracker.getMapName(),
										pose_name: newLabel
									});
									self.hasPoseService.callService(request, function (result) {
										if (!result.result) {
											self.changeTracker.applyPoseChange("rename", target.textContent,
												undefined, undefined, undefined, newLabel);
											target.textContent = newLabel;
										} else {
											showPopup(self.helpPopup, self.helpPopupContent, self.disableDiv,
												"The name \"" + newLabel + "\" already exists!");
										}
									});
								} else {
									self.changeTracker.applyPoseChange("rename", target.textContent,
												undefined, undefined, undefined, newLabel);
									target.textContent = newLabel;
								}
							} else if (labelType === "region_annotation" && !self.changeTracker.hasRegion(newLabel)) {
								if (self.dbConnected()) {
									// check if the region already exists in the database
									let request = new ROSLIB.ServiceRequest({
										map_name: self.changeTracker.getMapName(),
										region_name: newLabel
									});
									self.hasRegionService.callService(request, function (result) {
										if (!result.result) {
											self.changeTracker.applyRegionChange("rename", target.textContent,
												undefined, undefined, undefined, newLabel);
											target.textContent = newLabel;
										} else {
											showPopup(self.helpPopup, self.helpPopupContent, self.disableDiv,
												"The name \"" + newLabel + "\" already exists!");
										}
									});
								} else {
									self.changeTracker.applyRegionChange("rename", target.textContent,
												undefined, undefined, undefined, newLabel);
									target.textContent = newLabel;
								}
							} else {
								showPopup(self.helpPopup, self.helpPopupContent, self.disableDiv,
									"The name \"" + newLabel + "\" already exists!");
							}
						}
					}
				}
			}
		});
	}

	dbConnected() {
		return window.document.getElementById("connectDb").innerText === "Disconnect from Database";
	}

	enterShapeDeleteMode(typeToDelete) {
		this.inShapeDeleteMode = true;
		this.typeToDelete = typeToDelete;
	}

	exitShapeDeleteMode() {
		this.inShapeDeleteMode = false;
		this.typeToDelete = "";
		let result = [];
		if (this.deletedRegionIds.length > 0) {
			result = this.deletedRegionIds;
			this.deletedRegionIds = [];
		}
		return result;
	}

	enterRegionEditor(target) {
		window.document.getElementById("regionShapeBtns").style.visibility = "visible";
		$(':button:not(.regionShapeBtn, .popupCloseBtn)').prop('disabled', true);
		// highlight the selected region
		this.highlightRegion(target);		
	}

	exitRegionEditor() {
		window.document.getElementById("regionShapeBtns").style.visibility = "hidden";
		$(':button:not(.regionShapeBtn, .popupCloseBtn)').prop('disabled', false);
		// de-highlight the selected region
		if (this.selectedRegion != null) {
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
		this.selectedRegion.style.fillOpacity = "0.5";
	}

	calculateNewOffset(oldX, oldY, currentOffsetX, currentOffsetY) {
		let x = currentOffsetX - this.offset.x;
		let y = currentOffsetY - this.offset.y;
		let angle = Math.atan2(y, x);
		let unzoomedDistance = this.editor.getUnzoomedLength(Math.sqrt(Math.pow(y, 2) + Math.pow(x, 2)));
		let newOffsetX = oldX + unzoomedDistance * Math.cos(angle);
		let newOffsetY = oldY + unzoomedDistance * Math.sin(angle);
		return {x: newOffsetX, y: newOffsetY};
	}
}