pdf:
	cd docs; \
	[ -f revnumber.txt ] && REVNUMBER='-a revnumber='"$$(cat revnumber.txt)" || unset REVNUMBER; \
	asciidoctor-pdf $$REVNUMBER \
	  -a pdf-theme=src_adoc/neorv32-theme.yml \
	  src_adoc/neorv32.adoc \
	  --out-file NEORV32.pdf

html:
	cd docs; \
	[ -f revnumber.txt ] && REVNUMBER='-a revnumber='"$$(cat revnumber.txt)" || unset REVNUMBER; \
	asciidoctor $$REVNUMBER \
	  src_adoc/index.adoc \
	  --out-file index.html

revnumber:
	if [ `git tag -l | grep nightly` ]; then git tag -d nightly; fi
	git describe --long --tags  | sed 's#\([^-]*-g\)#r\1#;' > docs/revnumber.txt
	cat docs/revnumber.txt

container: revnumber
	docker run --rm -v /$(PWD)://documents/ asciidoctor/docker-asciidoctor make pdf html
