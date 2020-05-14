class Editor {
	constructor() {
		this.svg = null;
		this.canvas = null;
		this.width = 0;
		this.height = 0;
		this.resolution = -1;
		this.origin = [0, 0];
	}

	setup(svg, canvas) {
		this.svg = svg;
		this.canvas = canvas;
	}

	getMidpointX() {
		return this.width / 2;
	}

	getMidpointY() {
		return this.height / 2;
	}

	setYAML(yamlFileContent) {
		this.resolution = yamlFileContent["resolution"];
		this.origin[0] = yamlFileContent["origin"][0];
		this.origin[1] = yamlFileContent["origin"][1];
	}

	setSVG(svgFileContent) {	
		this.width = svgFileContent.documentElement.viewBox.baseVal.width;
		this.height = svgFileContent.documentElement.viewBox.baseVal.height;
		this.setSVGDimension();
		this.svg.innerHTML = svgFileContent.documentElement.innerHTML;
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
				if (lineProcessed === 2) {
					this.width = currentLine.split(" ")[0];
					this.height = currentLine.split(" ")[1];
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
		let imageData = new ImageData(this.width, this.height);
		imageData.data.set(dataArr);
		let context = this.canvas.getContext('2d');
		this.canvas.width = imageData.width;
		this.canvas.height = imageData.height;
		context.putImageData(imageData, 0, 0);
		// link the PGM to the SVG stage
		let pgmImage = document.createElementNS('http://www.w3.org/2000/svg', "use");
		pgmImage.setAttribute("x", 0);
		pgmImage.setAttribute("y", 0);
		pgmImage.setAttribute("width", imageData.width);
		pgmImage.setAttribute("height", imageData.height);
		pgmImage.setAttribute("href", "#canvas");
		this.svg.appendChild(pgmImage);
		this.setSVGDimension();
	}

	addElement(element) {
		this.svg.appendChild(element);
		this.svg.appendChild(document.createTextNode('\n'));
	}

	deleteElement(element) {
		this.svg.removeChild(element);
	}

	deleteElementOfGroup(g, element) {
		g.removeChild(element);
	}

	clear() {
		if (this.svg) {
			this.svg.textContent = "";
		}
		let context = this.canvas.getContext('2d');
		context.clearRect(0, 0, this.canvas.width, this.canvas.height);;
		this.width = "";
		this.height = "";
		this.resolution = -1;
		this.origin = [0, 0];
	}

	toString() {
		return [
			'<?xml version="1.0" encoding="UTF-8"?>\n',
			'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ' + this.width + ' ' + this.height + '\">\n',
			this.svg.innerHTML,
			'</svg>'
		].join('');
	}

	setSVGDimension() {
		this.svg.setAttribute("width", this.width);
		this.svg.setAttribute("height", this.height);
		this.svg.setAttribute("viewBox", "0 0 " + this.width + " " + this.height);
	}
}
