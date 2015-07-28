#!/bin/sh
echo '?xml version="1.0"?>'
echo '<opencv_storage><images>' 
ls $1
echo '</images></opencv_storage>'
echo
