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

		let offset = { x: 0, y: 0 };
		let angleOffset = 0;  // angle offset for pose orientation (in radians)
		this.selectedRegion = null;
		this.deletedRegionIds = [];
		let regionReferencePoint = { x: 0, y: 0 };

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
				angleOffset = Math.atan2(y2 - y1, x2 - x1);
				displayPoseInfo(x1, y1, angleOffset);
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
				if (targetType === 'circle_annotation' && self.selectedRegion == null) {

					offset.x = parseFloat(target.getAttribute('cx')) - event.clientX;
					offset.y = parseFloat(target.getAttribute('cy')) - event.clientY;

				} else if (targetType === 'pose_line_annotation' && self.selectedRegion == null) {
					let x1 = target.getAttribute('x1');
					let y1 = target.getAttribute('y1');
					offset.x = parseFloat(x1) - event.clientX;
					offset.y = parseFloat(y1) - event.clientY;
					angleOffset = Math.atan2(target.getAttribute('y2') - y1, target.getAttribute('x2') - x1);
				} else if (targetType === 'region_annotation') {
					let referenceGroup = target.parentElement;
					let referencePointElement = referenceGroup.childNodes[2];
					let translate = getTranslate(referenceGroup.getAttribute('transform'));
					regionReferencePoint.x = parseFloat(referencePointElement.getAttribute('cx'));
					regionReferencePoint.y = parseFloat(referencePointElement.getAttribute('cy'));
					offset.x = regionReferencePoint.x + translate[0] - event.clientX;
					offset.y = regionReferencePoint.y + translate[1] - event.clientY;
				} else if (targetType === 'region_endpoint_annotation') {
					offset.x = parseFloat(target.getAttribute('cx')) - event.clientX;
					offset.y = parseFloat(target.getAttribute('cy')) - event.clientY;
				}
				self.selected = target;
			}
		});

		window.addEventListener('mousemove', function (event) {
			if (self.selected) {
				let targetType = self.selected.getAttribute('class');
				let label = getLabelElement(self.selected);

				let newOffsetX = event.clientX + offset.x;
				let newOffsetY = event.clientY + offset.y;

				if (targetType === 'circle_annotation' && self.selectedRegion == null) {
					self.selected.setAttribute('cx', newOffsetX);
					self.selected.setAttribute('cy', newOffsetY);
					displayCircleInfo(newOffsetX, newOffsetY);
					label.setAttribute('x', newOffsetX);
					label.setAttribute('y', newOffsetY + labelPadding);
					self.changeTracker.applyPointChange("save", label.textContent, newOffsetX, newOffsetY);
				} else if (targetType === 'pose_line_annotation' && self.selectedRegion == null) {
					if (!event.shiftKey) {  // move pose
						self.selected.setAttribute('x1', newOffsetX);
						self.selected.setAttribute('y1', newOffsetY);
						label.setAttribute('x', newOffsetX);
						label.setAttribute('y', newOffsetY + lineLength + labelPadding);
					} else {  // shift key pressed, change the arrow orientation
						let x1 = parseFloat(self.selected.getAttribute('x1'));
						let y1 = parseFloat(self.selected.getAttribute('y1'));
						angleOffset = Math.atan2(newOffsetY - y1, newOffsetX - x1);
						newOffsetX = x1;
						newOffsetY = y1;
					}
					self.selected.setAttribute('x2', newOffsetX + lineLength * Math.cos(angleOffset));
					self.selected.setAttribute('y2', newOffsetY + lineLength * Math.sin(angleOffset));
					displayPoseInfo(newOffsetX, newOffsetY, angleOffset);
					self.changeTracker.applyPoseChange("save", label.textContent, newOffsetX, newOffsetY, angleOffset);
				} else if (targetType === 'region_annotation') {
					let newTranslateX = newOffsetX - regionReferencePoint.x;
					let newTranslateY = newOffsetY - regionReferencePoint.y;
					self.selected.parentElement.setAttribute('transform', 'translate(' + newTranslateX + ',' + newTranslateY + ')');
					// always update the list of points and translate
					let currentRegion = self.selected.parentElement.childNodes[1];
					let points = convertToList(currentRegion.getAttribute('points'));
					self.changeTracker.applyRegionChange("translate", label.textContent, points, newTranslateX, newTranslateY);
				} else if (targetType === 'region_endpoint_annotation') {
					// move the endpoint
					self.selected.setAttribute('cx', newOffsetX);
					self.selected.setAttribute('cy', newOffsetY);
					displayCircleInfo(newOffsetX, newOffsetY);
					// adjust the line
					let currentRegion = self.selected.parentElement.childNodes[1];
					let points = convertToList(currentRegion.getAttribute('points'));
					let selectedEndpointId = parseInt(self.selected.getAttribute('id').split("-")[1]);
					points[selectedEndpointId] = [newOffsetX, newOffsetY];
					currentRegion.setAttribute('points', convertToString(points));
					// track change
					self.changeTracker.moveRegionEndpoint(label.textContent, selectedEndpointId, newOffsetX, newOffsetY);
					// always update the translate
					let translate = getTranslate(self.selected.parentElement.getAttribute('transform'));
					self.changeTracker.applyRegionChange("translate", label.textContent, points, translate[0], translate[1]);
				}
				updateSelection(self.selected);
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
}