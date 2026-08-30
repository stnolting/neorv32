.DEFAULT_GOAL := help

PUBLIC_DIR := public
PUBLIC_IMG_DIR := $(PUBLIC_DIR)/img
PUBLIC_PDF_DIR := $(PUBLIC_DIR)/pdf
PUBLIC_UG_DIR := $(PUBLIC_DIR)/ug

DOXYGEN_BUILD_DIR := doxygen_build
DOXYGEN_IMG_DIR := $(DOXYGEN_BUILD_DIR)/html/docs/figures

.PHONY: all site pdf html ug-pdf ug-html doxygen
.PHONY: public-images doxygen-images revnumber clean help

all: pdf html ug-pdf ug-html

# Generate the revision number first and then build all AsciiDoc documents
site: revnumber
	$(MAKE) all

# Output directories
$(PUBLIC_DIR):
	@mkdir -p $@

$(PUBLIC_IMG_DIR): | $(PUBLIC_DIR)
	@mkdir -p $@

$(PUBLIC_PDF_DIR): | $(PUBLIC_DIR)
	@mkdir -p $@

$(PUBLIC_UG_DIR): | $(PUBLIC_DIR)
	@mkdir -p $@

$(DOXYGEN_IMG_DIR):
	@mkdir -p $@

# Copy images for datasheet and user guide
public-images: $(PUBLIC_IMG_DIR)
	cp -a figures/. $(PUBLIC_IMG_DIR)/

# Copy images required by Doxygen
doxygen-images: $(DOXYGEN_IMG_DIR)
	cp -a figures/. $(DOXYGEN_IMG_DIR)/

# Generate PDF datasheet
pdf: $(PUBLIC_PDF_DIR)
	@set -eu; \
	if [ -f revnumber.txt ]; then \
	  set -- -a "revnumber=$$(cat revnumber.txt)"; \
	else \
	  set --; \
	fi; \
	asciidoctor-pdf "$$@" \
	  -a allow-uri-read \
	  -a pdf-theme=neorv32-theme.yml \
	  -r asciidoctor-diagram \
	  datasheet/main.adoc \
	  --out-file $(PUBLIC_PDF_DIR)/NEORV32.pdf

# Generate HTML datasheet
html: public-images
	@set -eu; \
	if [ -f revnumber.txt ]; then \
	  set -- -a "revnumber=$$(cat revnumber.txt)"; \
	else \
	  set --; \
	fi; \
	asciidoctor "$$@" \
	  -r asciidoctor-diagram \
	  datasheet/index.adoc \
	  --out-file $(PUBLIC_DIR)/index.html

# Generate PDF user guide
ug-pdf: $(PUBLIC_PDF_DIR)
	@set -eu; \
	if [ -f revnumber.txt ]; then \
	  set -- -a "revnumber=$$(cat revnumber.txt)"; \
	else \
	  set --; \
	fi; \
	asciidoctor-pdf "$$@" \
	  -a allow-uri-read \
	  -a pdf-theme=neorv32-theme.yml \
	  -r asciidoctor-diagram \
	  userguide/main.adoc \
	  --out-file $(PUBLIC_PDF_DIR)/NEORV32_UserGuide.pdf

# Generate HTML user guide
ug-html: public-images $(PUBLIC_UG_DIR)
	@set -eu; \
	if [ -f revnumber.txt ]; then \
	  set -- -a "revnumber=$$(cat revnumber.txt)"; \
	else \
	  set --; \
	fi; \
	asciidoctor "$$@" \
	  -r asciidoctor-diagram \
	  userguide/index.adoc \
	  --out-file $(PUBLIC_UG_DIR)/index.html

# Generate Doxygen software documentation
doxygen: doxygen-images
	doxygen Doxyfile

# Generate revnumber.txt without modifying local Git tags
revnumber:
	git describe --long --tags | sed 's#\([^-]*-g\)#r\1#;' > revnumber.txt
	cat revnumber.txt

clean:
	@rm -rf $(PUBLIC_DIR)
	@rm -rf $(DOXYGEN_BUILD_DIR)
	@rm -f revnumber.txt

help:
	@echo "Targets:"
	@echo " all       - build datasheet and user guide as PDF and HTML"
	@echo " site      - generate revnumber.txt and build all AsciiDoc documents"
	@echo " pdf       - build datasheet PDF"
	@echo " html      - build datasheet HTML"
	@echo " ug-pdf    - build user guide PDF"
	@echo " ug-html   - build user guide HTML"
	@echo " doxygen   - build Doxygen software documentation"
	@echo " revnumber - generate revnumber.txt from the current Git revision"
	@echo " clean     - delete generated files and directories"
	@echo " help      - show this text"
