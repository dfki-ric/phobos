init:
		pip install -r requirements.txt

install:
		python3 setup.py

clean:
		rm python_dist_packages.conf

.PHONY: init test install
