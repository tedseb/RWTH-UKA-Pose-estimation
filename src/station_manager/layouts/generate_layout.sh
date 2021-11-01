#!/bin/bash
script=`realpath $0`
parentdir="$(dirname "$script")"
layout_qt_path="$(dirname "$parentdir")"/layouts
layout_python_path="$(dirname "$parentdir")"/src

array=("video_selection_ui" "station_selection_ui")

for i in "${array[@]}"; do   # The quotes are necessary here
    ui_file=${layout_qt_path}/${i}.ui
    py_file=${layout_python_path}/layouts/${i}.py
    pyuic5 -x $ui_file -o $py_file
done