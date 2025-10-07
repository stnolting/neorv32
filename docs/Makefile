.DEFAULT_GOAL := help

PUBLIC_DIR = public
PUBLIC_IMG_DIR = $(PUBLIC_DIR)/img

DOXYGEN_IMG_DIR = doxygen_build/html/docs/figures

all: pdf html ug-pdf ug-html

# Public folder
$(PUBLIC_DIR):
	@mkdir -p $(PUBLIC_DIR)

# Copy images for datasheet and ug html
$(PUBLIC_IMG_DIR): $(PUBLIC_DIR)
	@mkdir -p $@
	cp -vr figures/* $@

# Copy images for doxygen
$(DOXYGEN_IMG_DIR):
	@mkdir -p $@
	cp -vr figures/* $@

# Generate PDF datasheet
pdf: $(PUBLIC_DIR)
	[ -f revnumber.txt ] && REVNUMBER='-a revnumber='"$$(cat revnumber.txt)" || unset REVNUMBER; \
	asciidoctor-pdf $$REVNUMBER \
	  -a allow-uri-read \
	  -a pdf-theme=neorv32-theme.yml \
	  -r asciidoctor-diagram \
	  datasheet/main.adoc \
	  --out-file $(PUBLIC_DIR)/pdf/NEORV32.pdf

# Generate HTML datasheet
html: $(PUBLIC_IMG_DIR) $(PUBLIC_DIR)
	[ -f revnumber.txt ] && REVNUMBER='-a revnumber='"$$(cat revnumber.txt)" || unset REVNUMBER; \
	asciidoctor $$REVNUMBER \
	  -r asciidoctor-diagram \
	  datasheet/index.adoc \
	  --out-file $(PUBLIC_DIR)/index.html

# Generate PDF user guide
ug-pdf: $(PUBLIC_DIR)
	[ -f revnumber.txt ] && REVNUMBER='-a revnumber='"$$(cat revnumber.txt)" || unset REVNUMBER; \
	asciidoctor-pdf $$REVNUMBER \
	  -a allow-uri-read \
	  -a pdf-theme=neorv32-theme.yml \
	  -r asciidoctor-diagram \
	  userguide/main.adoc \
	  --out-file $(PUBLIC_DIR)/pdf/NEORV32_UserGuide.pdf

# Generate HTML user guide
ug-html: $(PUBLIC_IMG_DIR) $(PUBLIC_DIR)
	[ -f revnumber.txt ] && REVNUMBER='-a revnumber='"$$(cat revnumber.txt)" || unset REVNUMBER; \
	asciidoctor $$REVNUMBER \
	  -r asciidoctor-diagram \
	  userguide/index.adoc \
	  --out-file $(PUBLIC_DIR)/ug/index.html

# Generate DOXYGEN software documentation
doxygen: $(DOXYGEN_IMG_DIR)
	doxygen Doxyfile

# Generate revnumber.txt for overriding the revnumber attribute in 'pdf' and/or 'html'
revnumber:
	if [ `git tag -l | grep nightly_release` ]; then git tag -d nightly_release; fi
	git describe --long --tags | sed 's#\([^-]*-g\)#r\1#;' > revnumber.txt
	cat revnumber.txt

# Build 'pdf' and 'html' in an 'asciidoctor-wavedrom' container
container: revnumber
	docker run --rm -v /$(shell pwd)://documents/ btdi/asciidoctor make all

clean:
	@rm -rf $(PUBLIC_IMG_DIR)
	@rm -rf $(PUBLIC_DIR)
	@rm -rf doxygen_build
	@rm -f revnumber.txt

# Help
help:
	@echo "Targets:"
	@echo " all       - build datasheet and user guide as pdf and HTML file"
	@echo " help      - show this text"
	@echo " pdf       - build datasheet as pdf file (public/pdf/NEORV32.pdf)"
	@echo " html      - build datasheet as HTML page (public/index.html)"
	@echo " ug-pdf    - build user guide as pdf file (public/pdf/NEORV32_UserGuide.pdf)"
	@echo " ug-html   - build user guide as HTML page (public/ug/index.html)"
	@echo " doxygen   - build software documentation as HTML page (doxygen_build/html/index.html)"
	@echo " revnumber - for overriding the revnumber attribute in 'pdf' and/or 'html'"
	@echo " container - Build 'pdf' and 'html' in an 'asciidoctor-wavedrom' container"
	@echo " clean     - delete output files and directories"
