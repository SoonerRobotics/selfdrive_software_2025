#!/bin/bash -x

if [ -d "$HOME/scr/sdenv/" ]; then
	echo "Activating existing environment"
else
	echo "Creating new environment"
	python3 -m venv "$HOME/scr/sdenv"
	echo "Environment created"
fi

activate() {
	. $HOME/scr/sdenv/bin/activate
	. /opt/ros/jazzy/setup.sh
}

activate
