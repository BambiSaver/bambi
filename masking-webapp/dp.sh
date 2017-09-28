#!/bin/bash

# call script in this console to define the ftp_user and ftp_password environment variables
. ./.dp-ftp-config.sh


# retrieve the absolute path of this script in a portable manner
BASE_DIR=$(cd $(dirname "$0") && pwd)

ftp_host="w0102f68.kasserver.com"
remote_dir="/masking.bambi.florian.world/"
local_dir=$BASE_DIR; # we consider here that the web site sources are sibling of this script

# user specific parameters must be set in calling environment to allow several ftp users to use it and to avoid password storage
if [[ -z "${ftp_user}" ]]; then
  echo "'ftp_user' must be set"
  exit 1;
fi

if [[ -z "${ftp_password}" ]]; then
  echo "'ftp_password' must be set"
  exit 1;
fi

# use lftp to synchronize the source with the FTP server for only modified files.
lftp -c "
#debug;
open ftp://${ftp_user}:${ftp_password}@${ftp_host}
lcd ${local_dir};
cd ${remote_dir};
mirror \
       --ignore-time \
       --reverse \
       --parallel=5 \
       --verbose \
       --exclude .git/ \
       --exclude .gitignore \
       --exclude-glob composer.* \
       --exclude-glob *.sh" || exit $?