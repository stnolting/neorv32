# Generate PDF datasheet
pdf:
	cd docs; \
	[ -f revnumber.txt ] && REVNUMBER='-a revnumber='"$$(cat revnumber.txt)" || unset REVNUMBER; \
	asciidoctor-pdf $$REVNUMBER \
	  -a pdf-theme=src_adoc/neorv32-theme.yml \
	  -r asciidoctor-diagram \
	  src_adoc/neorv32.adoc \
	  --out-file NEORV32.pdf

# Generate HTML datasheet
html:
	cd docs; \
	[ -f revnumber.txt ] && REVNUMBER='-a revnumber='"$$(cat revnumber.txt)" || unset REVNUMBER; \
	asciidoctor $$REVNUMBER \
	  -r asciidoctor-diagram \
	  src_adoc/index.adoc \
	  --out-file index.html

# Generate revnumber.txt for overriding the revnumber attribute in 'pdf' and/or 'html'
revnumber:
	if [ `git tag -l | grep nightly` ]; then git tag -d nightly; fi
	git describe --long --tags | sed 's#\([^-]*-g\)#r\1#;' > docs/revnumber.txt
	cat docs/revnumber.txt

# Build 'pdf' and 'html' in an 'asciidoctor-wavedrom' container
container: revnumber
	docker run --rm -v /$(PWD)://documents/ btdi/asciidoctor make pdf html
