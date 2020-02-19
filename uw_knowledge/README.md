# uw_knowledge

Files storing knowledge about the environment. Maps, knowledgebase data. Scripts for creating, loading and manipulating this data.

## Usage

### make_map

This script runs hector_mapping with a reasonable configuration and crops the result. NOTE: hector_mapping expects the laser frame to be flipped from what the Fetch description has, so the map will need to be flipped vertically in Gimp. This can probably be fixed by publishing a custom flipped frame and relabeling the frame on the scan messages using topic_tools.
