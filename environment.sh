#!/bin/bash -x

if [ -d "~/scr/sdenv/" ]; then
	echo "Activating existing environment"
else
	echo "Creating new environment"
	python3 -m venv "$HOME/scr/sdenv"
	echo "Environment created"
fi

activate() {
	. $HOME/scr/sdenv/bin/activate
}

activate
