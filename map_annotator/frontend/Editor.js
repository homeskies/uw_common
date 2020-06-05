class Editor {
	constructor() {
		this.svg = null;
		this.canvas = null;
		this.pixelWidth = 0;
		this.pixelHeight = 0;
		this.resolution = 0;
		this.origin = {x: 0, y: 0};
		this.mapOriginY = 0;
		this.isReady = false;
		this.panZoomStage = null;
		this.isPanEnabled = false;
		// zoom and pan scale to make the image fill the editor canvas
		this.scaleToFill = 1;
		this.panToFill = {x: 0, y: 0};
	}

	setup(svg, canvas) {
		this.svg = svg;
		this.canvas = canvas;
	}

	isReadyToUse() {
		return this.isReady;
	}

	getOffset() {
		let svgDomRect = this.svg.getBoundingClientRect();
		return {x: svgDomRect["left"], y: svgDomRect["top"]};
	}

	getMidpointX() {
		return this.pixelWidth / 2;
	}

	getMidpointY() {
		return this.pixelHeight / 2;
	}

	getMapCoordinate(pixelX, pixelY) {
		let mapX = pixelX * this.resolution - Math.abs(this.origin.x);
		let mapY = - (pixelY * this.resolution - Math.abs(this.mapOriginY));
		return [mapX, mapY];
	}

	getPixelCoordinate(mapX, mapY) {
		let pixelX = (mapX + Math.abs(this.origin.x)) / this.resolution;
		let pixelY = - (mapY - Math.abs(this.mapOriginY)) / this.resolution;
		return [pixelX, pixelY];
	}

	getUnzoomedLength(zoomedLength) {
		return zoomedLength / this.panZoomStage.getZoom();
	}

	getTransformedCoordinate(x, y) {
		let m = getTransformMatrix(document.querySelector(".svg-pan-zoom_viewport").getAttribute('transform'));
		let transformedX = m[0] * x + m[2] * y + m[4];
		let transformedY = m[1] * x + m[3] * y + m[5];
		return [transformedX, transformedY];
	}

	setYAML(yamlFileContent) {
		this.resolution = yamlFileContent["resolution"];
		this.origin.x = yamlFileContent["origin"][0];
		this.origin.y = yamlFileContent["origin"][1];
	}

	setSVG(svgFileContent) {	
		this.setDimension(svgFileContent.documentElement.viewBox.baseVal.width, svgFileContent.documentElement.viewBox.baseVal.height);
		this.svg.innerHTML = svgFileContent.documentElement.innerHTML;
		this.isReady = true;
		this.enablePanZoom();
	}

	setPGM(pgmFileContent) {
		// convert the pgmFileContent array to string to get the PGM width and height
		let uint8array = new Uint8Array(pgmFileContent);
		let lines = new TextDecoder().decode(uint8array).split('\n');
		let lineProcessed = 0, charProcessed = 0, i = 0;
		while (lineProcessed < 3) {
			let currentLine = lines[i];
			if (!currentLine.startsWith("#")) {
				lineProcessed++;
				if (lineProcessed === 2) {  // update image dimension
					this.setDimension(currentLine.split(" ")[0], currentLine.split(" ")[1]);
				}
			}
			charProcessed += currentLine.length + 1;
			i++;
		}
		// process the pixel array
		let pixelArr = uint8array.slice(charProcessed);
		let dataArr = [];
		for (let i = 0; i < pixelArr.length; i++) {
			for (let j = 0; j < 3; j++) {  // r, g, b
				dataArr.push(pixelArr[i]);
			}
			dataArr.push(255);  // a
		}
		// draw the PGM image on the canvas
		let imageData = new ImageData(this.pixelWidth, this.pixelHeight);
		imageData.data.set(dataArr);
		let context = this.canvas.getContext('2d');
		this.canvas.width = imageData.width;
		this.canvas.height = imageData.height;
		context.putImageData(imageData, 0, 0);

		// get base64 encoded PNG data url from canvas
		let imgDataUrl = this.canvas.toDataURL("image/png");
		// link the PNG to the SVG stage
		let svgImg = document.createElementNS("http://www.w3.org/2000/svg", "image");
		svgImg.id = "background_img";
		svgImg.setAttributeNS("http://www.w3.org/1999/xlink", "xlink:href", imgDataUrl);
		this.svg.appendChild(svgImg);

		this.isReady = true;
		this.enablePanZoom();
	}

	enablePanZoom() {
        this.panZoomStage = svgPanZoom("#stage", {
			// uncomment this to enable built-in zoom control buttons
			// controlIconsEnabled: true,
			panEnabled: false,
			onZoom: function(newZoom) {
				document.getElementById("selection").style.display = "none";
			}, 
			onPan: function(newPan) {
				document.getElementById("selection").style.display = "none";
			}}
		);
		this.disablePan();
		this.fill();
	}

	fill() {
		// zoom and pan the image to fill the editor canvas
		if (this.panZoomStage) {
			this.panZoomStage.reset();
			this.panZoomStage.pan({x: this.panToFill.x, y: this.panToFill.y});
			this.panZoomStage.zoomBy(this.scaleToFill);
		}
	}

	isEditorPanEnabled() {
		return this.isPanEnabled;
	}

	enablePan() {
		if (this.panZoomStage) {
			this.panZoomStage.enablePan();
			this.isPanEnabled = true;
			document.getElementById("panSwitchBtn").innerHTML = "Pan ON";
		}
	}

	disablePan() {
		if (this.panZoomStage) {
			this.panZoomStage.disablePan();
			this.isPanEnabled = false;
			document.getElementById("panSwitchBtn").innerHTML = "Pan OFF";
		}
	}

	addElement(element) {
		document.querySelector(".svg-pan-zoom_viewport").appendChild(element);
		document.querySelector(".svg-pan-zoom_viewport").appendChild(document.createTextNode('\n'));
	}

	deleteElement(element) {
		document.querySelector(".svg-pan-zoom_viewport").removeChild(element);
	}

	deleteElementOfGroup(g, element) {
		g.removeChild(element);
	}

	resizeCircles(newRadius) {
		let circles = document.querySelectorAll("circle");
		for (let i = 0; i < circles.length; i++) {
			circles[i].setAttribute('r', newRadius);
		}
	}

	resizeLines(newWidth) {
		let lines = document.querySelectorAll("line, polygon");
		for (let i = 0; i < lines.length; i++) {
			lines[i].style.strokeWidth = newWidth;
		}
	}

	cloneStage() {
		// create a clone of the current svg
		return this.svg.cloneNode(true);
	}

	clearAnnotations() {
		let svgContent = document.querySelector(".svg-pan-zoom_viewport");
		let svgChildNodes = svgContent.childNodes;
		for (let i = 1; i < svgChildNodes.length; i++) {
			svgContent.removeChild(svgChildNodes[i]);
		}
	}

	clear() {
		if (this.panZoomStage) {
			this.panZoomStage.destroy();
			this.panZoomStage = null;
		}
		if (this.svg) {
			this.svg.textContent = "";
		}
		let context = this.canvas.getContext('2d');
		context.clearRect(0, 0, this.canvas.width, this.canvas.height);;
		this.pixelWidth = 0;
		this.pixelHeight = 0;
		this.resolution = 0;
		this.origin = {x: 0, y: 0};
		this.mapOriginY = 0;
		this.isReady = false;
		this.scaleToFill = 1;
		this.panToFill = {x: 0, y: 0};
		document.getElementById("selection").style.display = "none";
	}

	toString(svg) {
		let tag = [
			'<?xml version="1.0" encoding="UTF-8"?>\n',
			'<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" preserveAspectRatio="xMidYMid meet" ',
			'width="' + this.pixelWidth + '" height="' + this.pixelHeight + '" ',
			'viewBox="0 0 ' + this.pixelWidth + ' ' + this.pixelHeight + '\">\n'
		].join('');
		// remove all the pan zoom code from svg, only save the background image and annotations
		let content = svg.querySelector(".svg-pan-zoom_viewport").innerHTML;
		return tag + content + '</svg>';
	}

	setDimension(pixelWidth, pixelHeight) {
		this.pixelWidth = pixelWidth;
		this.pixelHeight = pixelHeight;
		this.mapOriginY = (this.pixelHeight - Math.abs(this.origin.y) / this.resolution) * this.resolution;
		// adjust the size of svg area so that the image won't appear too small or too large
		this.scaleToFill = Math.min((0.75 * window.innerWidth) / this.pixelWidth, (0.6 * window.innerHeight) / this.pixelHeight);
		let displayWidth = this.pixelWidth * this.scaleToFill;
		let displayHeight = this.pixelHeight * this.scaleToFill;
		this.panToFill.x = (displayWidth / 2) - (this.pixelWidth / 2);
		this.panToFill.y = (displayHeight / 2) - (this.pixelHeight / 2);
		this.svg.setAttribute("width", displayWidth);
		this.svg.setAttribute("height", displayHeight);
		this.svg.setAttribute("viewBox", "0 0 " + displayWidth + " " + displayHeight);
	}
}
