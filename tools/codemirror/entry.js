// The editor the coding server's Edit tab uses, bundled into one file so it can be served from
// the host itself: teammates are on the robot's wifi, which has no internet.
// Everything the page reaches for is on window.CM; the page test checks each name is here.
import { basicSetup } from "codemirror";
import { EditorView, keymap } from "@codemirror/view";
import { EditorState, Compartment } from "@codemirror/state";
import { indentWithTab } from "@codemirror/commands";
import { indentUnit } from "@codemirror/language";
import { setDiagnostics, lintGutter } from "@codemirror/lint";
import { java } from "@codemirror/lang-java";
import { oneDark } from "@codemirror/theme-one-dark";

window.CM = { basicSetup, EditorView, EditorState, Compartment, keymap, indentWithTab, indentUnit, setDiagnostics, lintGutter, java, oneDark };
