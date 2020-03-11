class Selector {
	constructor(stage, editor, lineLength, labelPadding) {
		this.REGION_HIGHLIGHT = "#ffe34c";

		var self = this;

		this.selected = null;
		this.inShapeDeleteMode = false;
		this.inEndpointDeleteMode = false;
		this.typeToDelete = "";
		this.editor = editor;
		var lineLength = lineLength;
		var labelPadding = labelPadding;

		var selection = document.createElement('span');
		selection.style.position = 'absolute';
		selection.style.display = 'none';
		selection.style.outline = 'solid 2px #99f';
		selection.style.pointerEvents = 'none';
		document.body.appendChild(selection);

		var offset = { x: 0, y: 0 };
		var angleOffset = 0;  // angle offset for pose orientation (in radians)
		this.selectedRegion = null;
		this.deletedRegionIds = [];
		var regionReferencePoint = { x: 0, y: 0 };

		function updateSelection(element) {
			if (element.isSameNode(stage)) {
				selection.style.display = 'none';
				return;
			}
			// highlight selection
			var rect = element.getBoundingClientRect();
			selection.style.left = (rect.left + window.pageXOffset) + 'px';
			selection.style.top = (rect.top + window.pageYOffset) + 'px';
			selection.style.width = rect.width + 'px';
			selection.style.height = rect.height + 'px';
			selection.style.display = 'block';
		}

		function displayCircleInfo(x, y) {
			document.getElementById("coordinateInfo").innerHTML = 
					"<p>X: " + x + "</p><p>Y: " + y + "</p>";
		}

		function displayPoseInfo(x, y, theta) {
			document.getElementById("coordinateInfo").innerHTML = 
					"<p>X: " + x + "</p><p>Y: " + y + 
					"<p>THETA: " + round(theta) + " rad</p>" +
					"<p>THETA: " + round(convertToDeg(theta)) + " deg</p>";
		}

		function clearCoordinateInfo() {
			document.getElementById("coordinateInfo").innerHTML = "";
		}

		// HOVER
		stage.addEventListener('mouseover', function (event) {
			var target = event.target;
			updateSelection(target);
			var targetType = target.getAttribute('class');
			if (targetType === 'circle_annotation' || targetType === 'region_endpoint_annotation') {
				displayCircleInfo(target.getAttribute('cx'), target.getAttribute('cy'));
			} else if (targetType === 'pose_line_annotation') {
				displayPoseInfo(target.getAttribute('x1'), target.getAttribute('y1'), angleOffset);
			}
		});

		stage.addEventListener('mouseout', function () {
			clearCoordinateInfo();
		});

		// DRAG & DROP
		stage.addEventListener('mousedown', function (event) {
			var target = event.target;
			if (target.isSameNode(stage) === false) {
				var targetType = target.getAttribute('class');
				if (targetType === 'circle_annotation') {
					offset.x = parseFloat(target.getAttribute('cx')) - event.clientX;
					offset.y = parseFloat(target.getAttribute('cy')) - event.clientY;
				} else if (targetType === 'pose_line_annotation') {
					var x1 = target.getAttribute('x1');
					var y1 = target.getAttribute('y1');
					offset.x = parseFloat(x1) - event.clientX;
					offset.y = parseFloat(y1) - event.clientY;
					angleOffset = Math.atan2(target.getAttribute('y2') - y1, target.getAttribute('x2') - x1);
				} else if (targetType === 'region_annotation') {
					var referenceGroup = target.parentElement;
					var referencePointElement = referenceGroup.childNodes[2];
					var translate = getTranslate(referenceGroup.getAttribute('transform'));
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
				var targetType = self.selected.getAttribute('class');
				var label = getLabelElement(self.selected);
				var newOffsetX = event.clientX + offset.x;
				var newOffsetY = event.clientY + offset.y;
				if (targetType === 'circle_annotation') {
					self.selected.setAttribute('cx', newOffsetX);
					self.selected.setAttribute('cy', newOffsetY);
					displayCircleInfo(newOffsetX, newOffsetY);
					label.setAttribute('x', newOffsetX);
					label.setAttribute('y', newOffsetY + labelPadding);
				} else if (targetType === 'pose_line_annotation') {
					if (event.shiftKey === false) {
						self.selected.setAttribute('x1', newOffsetX);
						self.selected.setAttribute('y1', newOffsetY);
						label.setAttribute('x', newOffsetX);
						label.setAttribute('y', newOffsetY + lineLength + labelPadding);
					} else {  // right click, change the arrow orientation
						var x1 = parseFloat(self.selected.getAttribute('x1'));
						var y1 = parseFloat(self.selected.getAttribute('y1'));
						angleOffset = Math.atan2(newOffsetY - y1, newOffsetX - x1);
						newOffsetX = x1;
						newOffsetY = y1;
					}
					self.selected.setAttribute('x2', newOffsetX + lineLength * Math.cos(angleOffset));
					self.selected.setAttribute('y2', newOffsetY + lineLength * Math.sin(angleOffset));
					displayPoseInfo(newOffsetX, newOffsetY, angleOffset);
				} else if (targetType === 'region_annotation') {
					var newTranslateX = newOffsetX - regionReferencePoint.x;
					var newTranslateY = newOffsetY - regionReferencePoint.y;
					self.selected.parentElement.setAttribute('transform', 'translate(' + newTranslateX + ',' + newTranslateY + ')');
				} else if (targetType === 'region_endpoint_annotation') {
					// move the endpoint
					self.selected.setAttribute('cx', newOffsetX);
					self.selected.setAttribute('cy', newOffsetY);
					displayCircleInfo(newOffsetX, newOffsetY);
					// adjust the line
					var currentRegion = self.selected.parentElement.childNodes[1];
					var points = convertToList(currentRegion.getAttribute('points'));
					var selectedEndpointId = parseInt(self.selected.getAttribute('id').split("-")[1]);
					points[selectedEndpointId] = [newOffsetX, newOffsetY];
					currentRegion.setAttribute('points', convertToString(points));
				}
				updateSelection(self.selected);
			}
		});

		stage.addEventListener('mouseup', function () {
			self.selected = null;
			clearCoordinateInfo();
		});

		stage.addEventListener('click', function (event) {
			var target = event.target;
			if (target.isSameNode(stage) === false) {
				var targetType = target.getAttribute('class');
				if (self.inShapeDeleteMode && targetType === self.typeToDelete) {
					// DELETE shape
					if (targetType === 'circle_annotation' || targetType === 'pose_line_annotation') {
						self.editor.deleteElement(target.parentElement);
					} else if (targetType === 'region_annotation') {
						self.deletedRegionIds.push(getRegionId(target));
						self.editor.deleteElement(target.parentElement);
					}
				} else if (targetType === 'region_annotation') {
					// edit selected region
					self.enterRegionEditor(target);
				} else if (self.inEndpointDeleteMode && event.shiftKey === true) {
					// DELETE endpoint
					var group = target.parentElement;
					var region = group.childNodes[1];
					var points = convertToList(region.getAttribute('points'));
					if (points.length === 3) {  // delete the entire region
						self.deletedRegionIds.push(getRegionId(group.childNodes[2]));
						self.editor.deleteElement(group);
					} else {  // delete the point
						self.editor.deleteElementOfGroup(group, target);
						var selectedEndpointId = parseInt(target.getAttribute('id').split("-")[1]);
						points.splice(selectedEndpointId, 1);
						region.setAttribute('points', convertToString(points));
						// update endpoint ids
						var regionId = getRegionId(region);
						for (var i = 2; i < group.childNodes.length; i++) {
							var newId = i - 2;
							group.childNodes[i].setAttribute('id', regionId + "-" + newId);
						}
					}
				} else if (targetType === 'text_annotation') {
					var newLabel = prompt("Please enter the label name:", target.textContent);
					if (newLabel != null && newLabel != "") {
						target.textContent = newLabel;
					}
				}
			}
		});
	}

	enterShapeDeleteMode(typeToDelete) {
		this.inShapeDeleteMode = true;
		this.typeToDelete = typeToDelete;
	}

	exitShapeDeleteMode() {
		this.inShapeDeleteMode = false;
		this.typeToDelete = "";
		var result = [];
		if (this.deletedRegionIds.length > 0) {
			result = this.deletedRegionIds;
			this.deletedRegionIds = [];
		}
		return result;
	}

	enterRegionEditor(target) {
		window.document.getElementById("regionShapeBtns").style.visibility = "visible";
		$(':button:not(.regionShapeBtn)').prop('disabled', true);
		// de-highlight the previously selected region
		if (this.selectedRegion) {
			this.selectedRegion.style.fill = 'transparent';
		}
		// highlight the selected region
		this.selectedRegion = target;
		this.selectedRegion.style.fill = this.REGION_HIGHLIGHT;
		this.selectedRegion.style.fillOpacity = "0.5";		
	}

	exitRegionEditor() {
		window.document.getElementById("regionShapeBtns").style.visibility = "hidden";
		$(':button:not(.regionShapeBtn)').prop('disabled', false);
		// de-highlight the selected region
		this.selectedRegion.style.fill = 'transparent';
		this.selectedRegion = null;
	}

	enterEndpointDeleteMode() {
		this.inEndpointDeleteMode = true;
	}

	exitEndpointDeleteMode() {
		this.inEndpointDeleteMode = false
	}

	getSelectedRegion() {
		return this.selectedRegion;
	}
}