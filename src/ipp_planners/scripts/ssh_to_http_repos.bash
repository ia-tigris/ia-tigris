#!/bin/bash
input=$1
# > newfile.repos
while IFS= read -r line
do
    # echo "$line"
    REPO_URL=`echo "$line" | sed -Ene's#.*(git@[^[:space:]]*).*#\1#p'`
    if [ -z "$REPO_URL" ]; then
        # echo "$line" >> newfile.repos
        echo "$line"
        continue
    fi

    REMOTE_URL=`echo $REPO_URL | sed -Ene's#git@([^/]*):(.*)/(.*).git#\1#p'`
    if [ -z "$REMOTE_URL" ]; then
        echo "-- ERROR:  Could not identify remote URL."
        exit
    fi

    USER=`echo $REPO_URL | sed -Ene's#git@'"$REMOTE_URL"':([^/]*)/(.*).git#\1#p'`
    if [ -z "$USER" ]; then
        echo "-- ERROR:  Could not identify user."
        exit
    fi

    REPO=`echo $REPO_URL | sed -Ene's#git@'"$REMOTE_URL"':([^/]*)/(.*).git#\2#p'`
    if [ -z "$REPO" ]; then
        echo "-- ERROR:  Could not identify Repo."
        exit
    fi

    NEW_URL="https://$REMOTE_URL/$USER/$REPO.git"

    # echo -e "\t\turl: $NEW_URL" >> newfile.repos
    echo -e "    url: $NEW_URL"
    
done < "$input"