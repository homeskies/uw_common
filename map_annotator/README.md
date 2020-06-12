# map_annotator
This is a browser-based editor for annotating maps. Users can upload a yaml file plus a map in pgm or svg format to annotate.

### To Launch the Web Interface:
1. Clone this repo;
2. In `map_annotator/frontend`, run: `$ npm install` to install additional pacakges needed for the interface.
3. In terminal, run: `$ python -m SimpleHTTPServer 8000`.
4. Visit [localhost:8000](localhost:8000).

### Usage
This interface has two modes:
* Offline Mode:  
This is the default mode. It lets you make annotations and download annotations in svg format.

* Database Mode:  
The interface connects to the [knowledge representation database](https://utexas-bwi.github.io/knowledge_representation/index.html), users can create new maps in the database, manage existing maps in the database, and make changes to an existing map by uploading the previous annotated map in svg format and editing it.  
To use the interface in database mode, in addition to launching the python server, you also need to:
  * Open a new terminal window and run `$ roslaunch map_annotator app.launch`. This command launches ROS websocket and the knowledge representation database.
  * On the web interface, click the top right button to connect/disconnect with the database.

### A Short Demo
<!-- blank line -->
<figure class="video_container">
  <iframe src="https://www.youtube.com/embed/y-9LvlOPW6E" frameborder="0" allowfullscreen="true"> </iframe>
</figure>
<!-- blank line -->

***
If you find any bugs or have any questions, please feel free to contact Xinyi Wang: wangx227@cs.washington.edu :)