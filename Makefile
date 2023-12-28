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




