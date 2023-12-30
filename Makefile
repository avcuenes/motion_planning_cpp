## Clean up the build project
.PHONY : clean 
clean:
	rm -rf build/ install/ log/ docs/html/  docs/latex/

.PHONY : docker_build
docker_build:
	docker build -t mpcpp .

.PHONY : docker_run
docker_run:
	docker run -it mpcpp 


.PHONY : doxygen_create
doxygen_create:
	doxygen Doxyfile


.PHONY : doxygen_viz
doxygen_viz:
	firefox docs/html/index.html



package_name ?= $(shell bash -c 'read -p "package name: " pwd; echo $$pwd')
node_name ?= $(shell bash -c 'read -p "node name: " pwd; echo $$pwd')


.PHONY : cretae_package 
cretae_package: $(SOURCE)
	@clear
	cd src && ros2 pkg create --build-type ament_cmake --node-name  $(package_name) $(node_name)


