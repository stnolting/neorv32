.DEFAULT_GOAL := help

all: pdf html ug-pdf ug-html
	mkdir -p public/img/
	cp -vr figures/* public/img/

# Generate PDF datasheet
pdf:
	[ -f revnumber.txt ] && REVNUMBER='-a revnumber='"$$(cat revnumber.txt)" || unset REVNUMBER; \
	asciidoctor-pdf $$REVNUMBER \
	  -a allow-uri-read \
	  -a pdf-theme=neorv32-theme.yml \
	  -r asciidoctor-diagram \
	  datasheet/main.adoc \
	  --out-file public/pdf/NEORV32.pdf

# Generate HTML datasheet
html:
	[ -f revnumber.txt ] && REVNUMBER='-a revnumber='"$$(cat revnumber.txt)" || unset REVNUMBER; \
	asciidoctor $$REVNUMBER \
	  -r asciidoctor-diagram \
	  datasheet/index.adoc \
	  --out-file public/index.html

# Generate PDF user guide
ug-pdf:
	[ -f revnumber.txt ] && REVNUMBER='-a revnumber='"$$(cat revnumber.txt)" || unset REVNUMBER; \
	asciidoctor-pdf $$REVNUMBER \
	  -a allow-uri-read \
	  -a pdf-theme=neorv32-theme.yml \
	  -r asciidoctor-diagram \
	  userguide/main.adoc \
	  --out-file public/pdf/NEORV32_UserGuide.pdf

# Generate HTML user guide
ug-html:
	[ -f revnumber.txt ] && REVNUMBER='-a revnumber='"$$(cat revnumber.txt)" || unset REVNUMBER; \
	asciidoctor $$REVNUMBER \
	  -r asciidoctor-diagram \
	  userguide/index.adoc \
	  --out-file public/ug/index.html

# Generate DOXYGEN software documentation
doxygen:
	doxygen Doxyfile

# Generate revnumber.txt for overriding the revnumber attribute in 'pdf' and/or 'html'
revnumber:
	if [ `git tag -l | grep nightly` ]; then git tag -d nightly; fi
	git describe --long --tags | sed 's#\([^-]*-g\)#r\1#;' > revnumber.txt
	cat revnumber.txt

# Build 'pdf' and 'html' in an 'asciidoctor-wavedrom' container
container: revnumber
	docker run --rm -v /$(shell pwd)://documents/ btdi/asciidoctor make all

# Help
help:
	@echo "Targets:"
	@echo " help    - show this text"
	@echo " pdf     - build datasheet as pdf file (public/pdf/NEORV32.pdf)"
	@echo " html    - build datasheet as HTML page (public/index.html)"
	@echo " ug-pdf  - build user guide as pdf file (public/pdf/NEORV32_UserGuide.pdf)"
	@echo " ug-html - build user guide as HTML page (public/ug/index.html)"
	@echo " doxygen - build software documentation as HTML page (doxygen_build/html/index.html)"
