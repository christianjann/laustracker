#!/bin/bash
set -v on

SOURCE=html/
DEST=christianjann@web.sourceforge.net:/home/project-web/liblaustracker/htdocs/

rsync -avz --progress --delete --delete-after -e ssh $SOURCE $DEST
