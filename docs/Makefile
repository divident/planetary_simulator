# Build the Two-Body Gravity documentation PDF.
#
# Requires a LaTeX toolchain; see the "Building this document" section of
# two-body.tex for per-platform installation instructions. Prefers latexmk,
# then tectonic, then plain pdflatex.

DOC = two-body

.PHONY: all clean watch

all: $(DOC).pdf

$(DOC).pdf: $(DOC).tex elliptical.png
	@if command -v latexmk >/dev/null 2>&1; then \
		echo "==> latexmk"; \
		latexmk -pdf -halt-on-error $(DOC).tex; \
	elif command -v tectonic >/dev/null 2>&1; then \
		echo "==> tectonic"; \
		tectonic $(DOC).tex; \
	elif command -v pdflatex >/dev/null 2>&1; then \
		echo "==> pdflatex (two passes)"; \
		pdflatex -halt-on-error $(DOC).tex && pdflatex -halt-on-error $(DOC).tex; \
	else \
		echo "No LaTeX toolchain found."; \
		echo "See the 'Building this document' section of $(DOC).tex."; \
		exit 1; \
	fi

# Rebuild continuously on save (latexmk only).
watch:
	latexmk -pdf -pvc $(DOC).tex

# Remove build intermediates, keeping the PDF.
clean:
	rm -f $(DOC).aux $(DOC).log $(DOC).out $(DOC).toc \
	      $(DOC).fls $(DOC).fdb_latexmk $(DOC).synctex.gz
