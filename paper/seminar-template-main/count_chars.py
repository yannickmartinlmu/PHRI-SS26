#!/usr/bin/env python3
"""Approximate texcount for content.tex and write chars.txt.

The template's \\charactercount macro shells out to texcount, which is not
installed here. This script strips LaTeX markup and counts the remaining body
text, then writes the "characters without spaces" figure to chars.txt so
main.tex compiles. Re-run with the real texcount if you have it.
"""
import re
import sys

SRC = sys.argv[1] if len(sys.argv) > 1 else "content.tex"
text = open(SRC, encoding="utf-8").read()

# drop comments
text = re.sub(r"(?<!\\)%.*", "", text)
# drop whole environments whose body is not prose
for env in ("tikzpicture", "CCSXML"):
    text = re.sub(rf"\\begin{{{env}}}.*?\\end{{{env}}}", "", text, flags=re.S)
# figure/subfigure wrappers: keep \caption text, drop the rest
text = re.sub(r"\\includegraphics(\[[^\]]*\])?\{[^}]*\}", "", text)
text = re.sub(r"\\Description\{(?:[^{}]|\{[^{}]*\})*\}", "", text)
text = re.sub(r"\\label\{[^}]*\}", "", text)
text = re.sub(r"\\(begin|end)\{[^}]*\}(\[[^\]]*\])?(\{[^}]*\})?", "", text)
# citations and refs count as a token or two of text
text = re.sub(r"\\(cite|autoref|ref)\{[^}]*\}", "XXXX", text)
# commands that wrap prose: keep the argument
text = re.sub(r"\\(emph|textit|textbf|texttt|paragraph|section|subsection|"
              r"subsubsection|title|caption)\*?(\[[^\]]*\])?\{", "{", text)
# any remaining command: drop it, keep braces content
text = re.sub(r"\\[a-zA-Z@]+\*?(\[[^\]]*\])?", " ", text)
text = re.sub(r"[{}$&~^_\\]", "", text)

words = len(text.split())
no_space = len(re.sub(r"\s", "", text))
with_space = len(" ".join(text.split()))

with open("chars.txt", "w") as fh:
    fh.write(str(no_space))

print(f"words            : {words}")
print(f"chars (no spaces): {no_space}")
print(f"chars (w/ spaces): {with_space}")
