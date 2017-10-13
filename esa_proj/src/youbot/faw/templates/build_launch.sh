#!/usr/bin/env bash
if [[ ! -z $2 ]]; then
	. $2
else
	. ./params.sh
fi

# Function from https://github.com/napsternxg/bash-template-engine
# Replaces tags (example: ${TAG}) with value in params.sh
render(){
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
    
    #find . -type f -name '*.o' -delete
    #find ${dir} -type f -name '*.template'
    
    shopt -s nullglob
	for f in `find ${dir} -maxdepth 1 -type f -name '*.template'` # For each template file in directory
	do
		filename="${f%.*}" # Get name without .template extension
		echo -e "Processing file $filename"
		
		# A nice header to identify generated files
		echo "<?xml version="1.0"?>" > ../launch/${filename}.launch
		printf "<!--Generate from source $f-->\n\n" >> ../launch/${filename}.launch
		# Run template engine on file and place in correct directory
		render "${filename}.template" >> ../launch/${filename}.launch
	done
done






