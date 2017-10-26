#!/usr/bin/env bash

#####################################################################################################
# 
# This file generates the ROS launch files in the launch directory of this package from the files in
# the templates directory. The script uses regex to replace common variables in launchfiles.
# It does this by detecting tags. A tag has the following signature:
#
# 	${TAG_NAME}
#
# The TAG_NAME is the also the key name of the key value pair values in the params.sh file. 
# Tags in this file have the following signature:
#
# 	TAG_NAME="string \$ to insert"
#
# Note that the dollar sign needs to be prefixed with a backslash. Otherwise the regex does not 
# correctly replace the key with the value.
#
# The script can handle folder structures. Files not in the root directory (templates) get the 
# include extension, not the launch extension. This means that includes from folders also need to
# have the correct extension.
#
# INSERT LICENSE
#####################################################################################################

# Set shell param #2 to external parameter file
# This is used by BASH_REMATCH.
. ./params.sh

# Function from https://github.com/napsternxg/bash-template-engine
# Replaces tags (example: ${TAG}) with value in params.sh
render()
{
	File="$1"
	while read -r line ; do
		if [[ $line == *"<?xml version="* ]]; then
			line = ""
		else
			while [[ "$line" =~ (\$\{[a-zA-Z_][a-zA-Z_0-9]*\}) ]] ; do
				LHS=${BASH_REMATCH[1]}
				RHS="$(eval echo "\"$LHS\"")"
				line=${line//$LHS/$RHS}
			done
			echo -e "$line"
		fi	
	done < $File
}

for dir in `find . -type d` # For each directory
do
    mkdir -p ../launch/${dir} # Create directory in launch if not exist
    
    fileExtension=".include"
    
    if [[ $dir == "." ]]; then
		fileExtension=".launch"
    fi
    
    #find . -type f -name '*.o' -delete
    #find ${dir} -type f -name '*.template'
    
    shopt -s nullglob
	for f in `find ${dir} -maxdepth 1 -type f -name '*.template'` # For each template file in directory
	do
		filename="${f%.*}" # Get name without .template extension
		echo -e "Processing file $filename"
		
		# A nice header to identify generated files
		echo "<?xml version=\"1.0\"?>" > ../launch/${filename}${fileExtension}
		printf "<!--Generate from source $f-->\n\n" >> ../launch/${filename}${fileExtension}
		# Run template engine on file and place in correct directory
		render "${filename}.template" >> ../launch/${filename}${fileExtension}
	done
done






