;; Style configuration

;; Remove emacs startup message (nil/t)
(setq inhibit-startup-screen nil)

;; Auto revert mode, load file automatically again if changed
(global-auto-revert-mode 1)

(setq-default c-basic-offset 8
              indent-tabs-mode t)

(setq-default
 tab-width 8
 )

;; No sound, visible
(setq visible-bell nil
      ring-bell-function 'flash-mode-line)
(defun flash-mode-line ()
  (invert-face 'mode-line))

;; UTF-8 everywhere (is probably the default already)
(set-default buffer-file-coding-system 'utf-8-unix)
(set-default-coding-systems 'utf-8-unix)
(prefer-coding-system 'utf-8-unix)
;; (set-default default-buffer-file-coding-system 'utf-8-unix)

;; No blink limit for cursor:
(setq blink-cursor-blinks 0)


;; Not sure what this is, comment it out
;; stop the silly emacs default scrolling
;; (setq scroll-step 1)
;; (setq hscroll-step 1)
;; (setq scroll-conservatively 50)
;; ;; start scrolling when 3 lines from top/bottom
;; (setq scroll-margin 3)
;; ;; move cursor, not buffer (?)
;; (setq scroll-preserve-screen-position 't)

;; Theme:
;; molokai theme linum style
(setq molokai-theme-kit t)

;; Not sure what this is for, test it
;; ;; disable menu/tool/scroll bars
;; ;;(when (fboundp 'menu-bar-mode)   (menu-bar-mode -1))
;; (when (fboundp 'tool-bar-mode)   (tool-bar-mode -1))
;; ;;(when (fboundp 'scroll-bar-mode) (scroll-bar-mode -1))
;; (set-scroll-bar-mode 'right)

;; Never used it
;; hide-show ifdef blocks
;; (global-set-key (kbd "<f3>") 'hide-ifdef-block)
;; (global-set-key (kbd "<S-f3>") 'show-ifdef-block)

;; Cool but I don't use it
;; highlight symbol
;; (global-set-key (kbd "<C-f4>") 'highlight-symbol-at-point)
;; (global-set-key (kbd "<f4>") 'highlight-symbol-next)
;; (global-set-key (kbd "<S-f4>") 'highlight-symbol-prev)

;; highlight FIXME, TODO, XXX, !!!
(defun my-highlight-fixme ()
  (interactive)
  (font-lock-mode 1)
  (font-lock-add-keywords
   nil '(("\\<\\(todo\\|FIXME\\|TODO\\|XXX\\|!!!\\)" 1 font-lock-warning-face prepend))))

;; jump cursor between matching parenthesis
(defun match-paren (arg)
  "Go to the matching parenthesis if on parenthesis otherwise insert %."
  (interactive "p")
  (cond ((looking-at "\\s\(") (forward-list 1) (backward-char 1))
        ((looking-at "\\s\)") (forward-char 1) (backward-list 1))
        (t (self-insert-command (or arg 1)))))


;; ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; ;;; special modes ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; ;; custom highlight ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(add-hook 'vhdl-mode-hook 'my-highlight-fixme)
(add-hook 'c-mode-common-hook 'my-highlight-fixme)
(add-hook 'cperl-mode-hook 'my-highlight-fixme)
(add-hook 'emacs-lisp-mode-hook 'my-highlight-fixme)
(add-hook 'LaTeX-mode-hook 'my-highlight-fixme)
(add-hook 'asm-mode-hook 'my-highlight-fixme)

;; ;; c ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(setq c-backslash-column 70)
(setq c-backslash-max-column 79)
(setq c-font-lock-extra-types (quote ("FILE" "\\sw+_pt" "\\sw+_tp" "\\sw+_t" "bool" "complex" "imaginary" "Lisp_Object" "I" "I1" "I2" "I4" "U" "U1" "U2" "U4" "L" "L1" "L2" "L4" "R4" "R8" "CH" "C2" "C4" "C8" "reg")))
(setq c-hanging-braces-alist (quote ((brace-list-open before after) (brace-entry-open before after) (statement-cont before after) (substatement-open before after) (block-close . c-snug-do-while) (extern-lang-open after) (namespace-open after) (module-open after) (composition-open after) (inexpr-class-open after) (inexpr-class-close before))))
(setq c-hanging-colons-alist (quote set-from-style))
(setq c-offsets-alist (quote ((substatement-open . 0))))
(setq c-style-variables-are-local-p nil)
(setq c-set-style "bsd")
(setq c-default-style "bsd")
(setq c-hungry-delete-key nil) ; hungry deleting sucks a lot
;; (add-hook 'c-mode-common-hook ; fix broken switch/case indentation
;;           (lambda () (c-set-offset 'case-label +)(linum-mode)))
;; (font-lock-add-keywords 'c-mode '(("\\<\\(\\sw+\\) ?(" 1 'font-lock-function-name-face)))

;; indentation (also for c++)
;; C-c C-s gives the label used below for a particular line, possible values are:
;; + (1 x basic offset), - (-1x), ++ (2x), -- (-2x), * (0.5x), / (-0.5x)
;; http://www.gnu.org/software/emacs/manual/html_mono/ccmode.html#Class-Symbols
;; http://www.gnu.org/software/emacs/manual/html_mono/ccmode.html#Customizing-Indentation
;; (setq c-basic-offset 4)
;; (c-set-offset 'case-label '+)
;; (c-set-offset 'inclass '+)
;; (c-set-offset 'access-label '-)
;; (c-set-offset 'topmost-intro '0)
;; (c-set-offset 'arglist-cont-nonempty '4)
;; (c-set-offset 'arglist-intro '4)
;; (c-set-offset 'statement-block-intro '4)
;; (c-set-offset 'defun-block-intro '4)

;; indentation
;; (setq indent-line-function 'insert-tab)

;; perl ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(setq cperl-auto-newline nil)
(setq cperl-brace-offset -2)
(setq cperl-electric-keywords nil)
(setq cperl-extra-newline-before-brace-multiline t)
;; (setq cperl-indent-comment-at-column-0 t)
;; (setq cperl-indent-level 4)
(setq cperl-invalid-face (quote trailing-whitespace))

;; Octave ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(setq auto-mode-alist (cons '("\\.m\\'" . octave-mode) auto-mode-alist))
(add-hook 'inferior-octave-mode-hook (lambda ()
                                       (turn-on-font-lock)
                                       (define-key inferior-octave-mode-map [up]   'comint-previous-input)
                                       (define-key inferior-octave-mode-map [down] 'comint-next-input)))

(setq octave-mode-hook
      (lambda () (progn (setq octave-comment-char ?%)
                        (setq comment-start "%")
                        (setq indent-tabs-mode t)
                        (setq comment-add 0)
                        (setq tab-width 2)
                        (setq tab-stop-list (number-sequence 2 200 2))
                        (setq octave-block-offset 2)

                        (defun octave-indent-comment ()
                          "A function for `smie-indent-functions' (which see)."
                          (save-excursion
                            (back-to-indentation)
                            (cond
                             ((octave-in-string-or-comment-p) nil)
                             ((looking-at-p "\\(\\s<\\)\\1\\{2,\\}") 0)))))))
;; css ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(autoload 'css-mode "css-mode")
(setq auto-mode-alist (cons '("\\.css\\'" . css-mode) auto-mode-alist))

;; php ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(autoload 'php-mode "php-mode")
(setq auto-mode-alist (cons '("\\.php\\'" . php-mode) auto-mode-alist))
(add-hook 'php-mode-hook (lambda () (c-set-style "linux")))

;; gnuplot ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(autoload 'gnuplot-mode "gnuplot" "gnuplot major mode" t)
(autoload 'gnuplot-make-buffer "gnuplot" "open a buffer in gnuplot mode" t)
(setq auto-mode-alist (append '(("\\.gp$" . gnuplot-mode)) auto-mode-alist))

;; ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; ;;; funky functions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; scroll text one line up/down while keeping the cursor
(defun scroll-up-keep-cursor () (interactive) (scroll-up 1))
(defun scroll-down-keep-cursor () (interactive) (scroll-down 1))

;; move one word forward / backward, leave cursor at start of word
;; instead of emacs default end of word, treat _ as part of word
(defun geosoft-forward-word () (interactive)
  (forward-char 1) (backward-word 1) (forward-word 2) (backward-word 1) (backward-char 1)
  (cond ((looking-at "_") (forward-char 1) (geosoft-forward-word)) (t (forward-char 1))))
(defun geosoft-backward-word () (interactive)
  (backward-word 1) (backward-char 1)
  (cond ((looking-at "_") (geosoft-backward-word)) (t (forward-char 1))))

;; functions to change line ending characters, like flip(1)
(defun flip-unix ()
  "Change the current buffer to Latin 1 with Unix line-ends."
  (interactive)
  (set-buffer-file-coding-system 'iso-latin-1-unix t))
(defun flip-dos ()
  "Change the current buffer to Latin 1 with DOS line-ends."
  (interactive)
  (set-buffer-file-coding-system 'iso-latin-1-dos t))
(defun flip-mac ()
  "Change the current buffer to Latin 1 with Mac line-ends."
  (interactive)
  (set-buffer-file-coding-system 'iso-latin-1-mac t))


;; delete happiness (from http://www.ludd.luth.se/~wilper-8/computer/emacs.html)
(defun delete-word (arg)
  "Delete characters forward until encountering the end of a word. With argument, do this that many times."
  (interactive "*p")
  (delete-region (point) (save-excursion (forward-word arg) (point))))
(defun backward-delete-word (arg)
  "Delete characters backward until encountering the end of a word. With argument, do this that many times."
  (interactive "*p")
  (delete-word (- arg)))

;; different cursor based on mode
;; (add-hook 'post-command-hook '(lambda ()
;;                                 (cond (buffer-read-only (set-cursor-color "grey") (setq cursor-type 'box))
;;                                       (overwrite-mode (set-cursor-color "red") (setq cursor-type 'hollow))
;;                                       (t (set-cursor-color "red") (setq cursor-type 'box)))))

;; highlight surrounding parentheses
(require 'highlight-parentheses)
(add-hook 'c-mode-hook (lambda () (highlight-parentheses-mode)))
(add-hook 'perl-mode-hook (lambda () (highlight-parentheses-mode)))
(add-hook 'cperl-mode-hook (lambda () (highlight-parentheses-mode)))
(add-hook 'emacs-lisp-mode-hook (lambda () (highlight-parentheses-mode)))

;; ;; highlight columns beyond the magic boundary
;; (require 'highlight-80+)
;; (add-hook 'c-mode-hook '(lambda () (highlight-80+-mode) (setq fill-column 100)))
;; (add-hook 'perl-mode-hook '(lambda () (highlight-80+-mode) (setq fill-column 100)))
;; (add-hook 'cperl-mode-hook '(lambda () (highlight-80+-mode) (setq fill-column 100)))

;; always turn on hi-fi font-lock
(require 'font-lock)
(if (fboundp 'global-font-lock-mode) (global-font-lock-mode t))
(setq font-lock-mode-maximum-decoration t)

(add-hook 'json-mode-hook
          (lambda ()
            (make-local-variable 'js-indent-level)
            (setq js-indent-level 2)))

(progn
  ;; Make whitespace-mode with very basic background coloring for whitespaces.
  ;; http://ergoemacs.org/emacs/whitespace-mode.html
  (setq whitespace-style (quote (face spaces tabs newline space-mark tab-mark newline-mark )))

  ;; Make whitespace-mode and whitespace-newline-mode use “¶” for end of line char and “▷” for tab.
  (setq whitespace-display-mappings
        ;; all numbers are unicode codepoint in decimal. e.g. (insert-char 182 1)
        '(
          (space-mark 32 [183] [46]) ; SPACE 32 「 」, 183 MIDDLE DOT 「·」, 46 FULL STOP 「.」
          (newline-mark 10 [182 10]) ; LINE FEED,
          (tab-mark 9 [8594 9] [92 9]) ; tab
          )))

;; tab width
;; (setq-default tab-width 8)
;; (setq tab-stop-list (number-sequence 4 120 4)

;; ;; spaces, not tabs
;; (setq-default indent-tabs-mode t)
;; (setq indent-tabs-mode nil)

;; ;; even for xml
;; (setq-default nxml-child-indent 4)
;; (setq-default nxml-attribute-indent 4)
;; allow S-cursor to mark a region (not really, rather: highlight marked region)
(setq transient-mark-mode t)

;; show size of buffer
(setq size-indication-mode t)

;; show column number
(setq column-number-mode t)

;; do not automatically add new lines at end of buffer (when pressing <down> at end of file)
(setq next-line-add-newlines nil)

;; show 24h clock and date
(display-time)
(setq-default display-time-day-and-date t)
(setq display-time-24hr-format t)

;; make C-k kill whole line instead of just to end of line (when at beginning of line)
(setq kill-whole-line t)

;; fringes (proper one at the right, small one left just for some padding)
(set-fringe-style (quote (2 . 8)))

;; remember where the cursor was in a file, store in ~/.emacs.d/places
(require 'saveplace)
(setq-default save-place t)
(setq save-place-file (concat user-emacs-directory "places"))

;; remember recently visited files (File->Open Recent), and save in ~/.emacs.d/recentf
(require 'recentf)
(recentf-mode 1)
(setq recentf-max-menu-items 25)
(setq recentf-save-file (concat user-emacs-directory "recentf"))

;; save desktop (open files) and load again on start, use only one desktop, save on file open, save more stuff with it
(require 'desktop)
(setq desktop-save-mode t)
(setq desktop-restore-eager 10)
(setq desktop-save t)
(setq desktop-load-locked-desktop t)
;; (add-to-list 'desktop-modes-not-to-save 'dired-mode)
;; (add-to-list 'desktop-modes-not-to-save 'Info-mode)
;; (add-to-list 'desktop-modes-not-to-save 'Apropos-mode)
;; (add-to-list 'desktop-modes-not-to-save 'info-lookup-mode)
;;                                         ;(add-to-list 'desktop-modes-not-to-save 'fundamental-mode) ; nope!
;; (setq desktop-path '("~/.emacs.d/"))
;; (setq desktop-dirname "~/.emacs.d/")
;;                                         ;(setq desktop-dirname user-emacs-directory)
;; (setq desktop-base-file-name "desktop")
;; (setq desktop-base-lock-name "desktop.lock")
;;                                         ;(add-hook 'find-file-hook (lambda () (run-with-timer 5 nil (lambda () (setq desktop-file-modtime (nth 5
;;                                         ;    (file-attributes (desktop-full-file-name)))) (desktop-save user-emacs-directory)))))
;; (setq desktop-globals-to-save (append '(
;;                                         (extended-command-history . 30) (file-name-history . 100) (grep-history . 30) (compile-history . 30)
;;                                         (minibuffer-history . 50) (query-replace-history . 60) (read-expression-history . 60)
;;                                         (regexp-history . 60) (regexp-search-ring . 20) (search-ring . 20) (shell-command-history . 50)
;;                                         (evil-ex .100) tags-file-name register-alist)))
;;                                         ;(add-hook 'kill-emacs-hook (lambda () (setq desktop-file-modtime (nth 5 (file-attributes (desktop-full-file-name))))))

;; make mouse avoid the cursor
;; (cond (window-system (require 'avoid) (mouse-avoidance-mode 'animate)))

;; Midnight mode. Cleans buffers every midnight
(require 'midnight)
(midnight-delay-set 'midnight-delay "4:30am")


;; make buffer names unique when visiting multiple files with the same name
(require 'uniquify)
(setq uniquify-buffer-name-style 'forward)

;; don't wrap long line
(set-default 'truncate-lines t)
(setq truncate-partial-width-windows nil)

;; set default fill-colum
(setq-default fill-column 80)

;; auto-fill-mode by default
;; (setq text-mode-hook 'turn-on-auto-fill)

;; highlight matching parenthese on cursor over
(show-paren-mode 1)

;; more extensive apropos searches
(setq apropos-do-all t)

;; kill and yank also fills the x clipboard rather than just the primary selection
(setq x-select-enable-clipboard t)
(setq x-select-enable-primary t)
(setq save-interprogram-paste-before-kill t)

;; use traditional selection behaviour
;; (pc-selection-mode t) ;; Emacs < 24
(delete-selection-mode t) ;; Emacs >= 24
(setq select-active-regions nil) ;; Emacs >= 24
(setq pc-select-selection-keys-only t)
;; (setq pc-select-meta-moves-sexps t)

(setq utf-translate-cjk-mode nil) ; disable CJK coding/encoding (Chinese/Japanese/Korean characters)
(set-language-environment 'utf-8)
(setq locale-coding-system 'utf-8)
(set-default-coding-systems 'utf-8)
(set-terminal-coding-system 'utf-8)
(unless (eq system-type 'windows-nt) (set-selection-coding-system 'utf-8))
(prefer-coding-system 'utf-8)
;; (standard-display-european t)
(set-keyboard-coding-system 'utf-8)

;; Disable bars
(menu-bar-mode -1)
(toggle-scroll-bar -1)
(tool-bar-mode -1)

;; Python comment offsets
(add-hook 'python-mode-hook
          (lambda () (set (make-local-variable 'comment-inline-offset) 2)))

(add-hook 'c++-mode-hook
          (lambda () (set (make-local-variable 'comment-inline-offset) 2)))

;; (add-hook 'c-mode-hook
;;            (lambda () (set (make-local-variable 'comment-inline-offset) 3)))

;; Org mode
;; org mode
(require 'org)
(define-key global-map "\C-cl" 'org-store-link)
(define-key global-map "\C-ca" 'org-agenda)
(setq org-log-done t)


;; (setq org-agenda-files (list "~/my-priv-things/org/work.org"))
;; (add-hook 'org-mode-hook
;;           (lambda ()
;;             (org-bullets-mode t)))

(setq org-hide-leading-stars t)

;; (setq org-src-fontify-natively t)

;; make scripts executable on save
(add-hook 'after-save-hook 'executable-make-buffer-file-executable-if-script-p)

;; make all yes/no prompts y/n instead
(fset 'yes-or-no-p 'y-or-n-p)

;; ask before quitting
(setq confirm-kill-emacs (quote y-or-n-p))

;; prefer cperl-mode over perl-mode
(defalias 'perl-mode 'cperl-mode)

;; make color escape sequences work in shell frames
(autoload 'ansi-color-for-comint-mode-on "ansi-color" nil t)
(add-hook 'shell-mode-hook 'ansi-color-for-comint-mode-on)

;; by default we start in text mode.
(setq initial-major-mode (lambda () (text-mode) (turn-on-auto-fill) (font-lock-mode)))

;; allow scrolling while in search mode ; FIXME: WTF?
(setq isearch-allow-scroll t)

;; visually indicate empty lines after end of file in the left fringe thingy
(setq indicate-empty-lines t)

;; never make new frames
(setq pop-up-frames nil)
(setq pop-up-windows t)

;; hide-ifdef use shadow instead of folding
(require 'hideif)
(add-hook 'c-mode-hook 'hide-ifdef-mode)
(setq hide-ifdef-shadow t)

;; tramp use SFTP by default
(require 'tramp)
                                        ;(setq tramp-default-method "sftp")
(setq tramp-default-method "ssh")

(defun delete-word-xxx (arg)
  "Delete characters forward until encountering the end of a word. With argument, do this that many times."
  (interactive "*p")
  (delete-region (point) (save-excursion (forward-word arg) (point))))

;; Zoom mode
(defun size-callback ()
  (cond ((> (frame-pixel-width) 1280) '(90 . 0.6))
        (t                            '(0.5 . 0.5))))

;; Ace window
(global-set-key (kbd "C-x o") 'ace-window)

(defun my-find-file-check-make-large-file-read-only-hook ()
  "If a file is over a given size, make the buffer read only."
  (when (> (buffer-size) (* 1024 1024))
    (setq buffer-read-only t)
    (buffer-disable-undo)
    (fundamental-mode)))

(add-hook 'find-file-hook 'my-find-file-check-make-large-file-read-only-hook)

;; autopair mode
(require 'autopair)
(autopair-global-mode 1)

;; C and C++
(add-hook 'c-mode-common-hook
          (lambda ()
            (when (derived-mode-p 'c-mode 'c++-mode 'java-mode)
              (ggtags-mode 1))))

;; SMEX -- autocomplete for M-x
(require 'smex) ; Not needed if you use package.el
(smex-initialize) ; Can be omitted. This might cause a (minimal) delay
                                        ; when Smex is auto-initialized on its first run.

(global-set-key (kbd "M-x") 'smex)
(global-set-key (kbd "M-X") 'smex-major-mode-commands)
;; This is your old M-x.
(global-set-key (kbd "C-c C-c M-x") 'execute-extended-command)

;; dired
(require 'dired+)

;; hippie expand M-/
(global-set-key (kbd "M-/") 'hippie-expand)

(setq hippie-expand-try-functions-list '(try-expand-dabbrev try-expand-dabbrev-all-buffers try-expand-dabbrev-from-kill try-complete-file-name-partially try-complete-file-name try-expand-all-abbrevs try-expand-list try-expand-line try-complete-lisp-symbol-partially try-complete-lisp-symbol))


;; Change kind of comments in c++. TURN ON IF WE ARE WRITING C++

;; open .h files in c++ mode
;; (add-to-list 'auto-mode-alist '("\\.h\\'" . c++-mode))
;; (add-hook 'c++-mode-hook (lambda () (setq comment-start "//"

;;                                          comment-end   "")))

;; Custom functions
(defun my-format-function()
  "Format buffer"
  (interactive)
  (untabify (point-min) (point-max))
  (delete-trailing-whitespace (point-min) (point-max))
  (indent-region (point-min) (point-max)))

(defun mrc-dired-do-command (command)
  "Run COMMAND on marked files. Any files not already open will be opened.
After this command has been run, any buffers it's modified will remain
open and unsaved."
  (interactive "CRun on marked files M-x ")
  (save-window-excursion
    (mapc (lambda (filename)
            (find-file filename)
            (call-interactively command))
          (dired-get-marked-files))))

;; font size
(set-face-attribute 'default nil :height 110)

;; Display line numbers
(global-linum-mode t)

;; show (ugly) trailing whitespace
(setq-default show-trailing-whitespace t)

;; Disable trailing whitespace in eshell and term mode
(add-hook 'eshell-mode-hook (lambda () (setq show-trailing-whitespace nil)))
(add-hook 'term-mode-hook (lambda () (setq show-trailing-whitespace nil)))
(add-hook 'shell-mode-hook (lambda () (setq show-trailing-whitespace nil)))


(provide 'my_style)
