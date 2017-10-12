#!/usr/bin/env bash
if [[ ! -z $2 ]]; then
	. $2
else
	. ./params.sh
fi

LBLUE='\033[1;34m'
NC='\033[0m' # No Color

render(){
	File="$1"
	while read -r line ; do
	    while [[ "$line" =~ (\$\{[a-zA-Z_][a-zA-Z_0-9]*\}) ]] ; do
		LHS=${BASH_REMATCH[1]}
		RHS="$(eval echo "\"$LHS\"")"
		line=${line//$LHS/$RHS}
	    done
	    echo -e "$line"
	done < $File
}


shopt -s nullglob
for f in *.template
do
	filename="${f%.*}"
	echo -e "Processing file $filename"
	
	echo "<!--Generate from source $f-->" >> ../launch/${filename}.launch
	render "${filename}.template" >> ../launch/${filename}.launch
done
