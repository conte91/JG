#!/bin/sh
echo '<?xml version="1.0"?>'
echo '<opencv_storage><images>' 
find $1 -type f -print
echo '</images></opencv_storage>'
echo
