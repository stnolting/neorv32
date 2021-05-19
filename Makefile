pdf:
	asciidoctor-pdf -a pdf-theme=docs/src_adoc/neorv32-theme.yml docs/src_adoc/neorv32.adoc --out-file docs/NEORV32.pdf

html:
	asciidoctor docs/src_adoc/index.adoc --out-file docs/index.html

container:
	docker run --rm -v /$(PWD)://documents/ asciidoctor/docker-asciidoctor make pdf html
