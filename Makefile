build:
	docker build -t sdk .

run:
	docker run -p 8080:8080 -p 8007:8007 -t sdk