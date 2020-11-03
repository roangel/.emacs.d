;; Style configuration

;; Remove emacs startup message (nil/t)
(setq inhibit-startup-screen t)

;; Auto revert mode, load file automatically again if changed
(global-auto-revert-mode 1)

(require 'my_guessed_styles)

;; No tabs, spaces
(setq-default
 indent-tabs-mode nil)

(setq-default
 tab-width 2)

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

;; Set default cursor to a bar of width 3
(setq cursor-type '(bar . 3))

;; Disable bars
(menu-bar-mode -1)
(scroll-bar-mode -1)
(tool-bar-mode -1)

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

;; highlight columns beyond the magic boundary
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
(set-fringe-style (quote (12 . 8)))

;; remember where the cursor was in a file, store in ~/.emacs.d/places
(require 'saveplace)
(setq-default save-place t)
(setq save-place-file (concat user-emacs-directory "places"))

;; remember recently visited files (File->Open Recent), and save in ~/.emacs.d/recentf
(require 'recentf)
(recentf-mode 1)
(setq recentf-max-menu-items 25)
(setq recentf-save-file (concat user-emacs-directory "recentf"))

;; Store buffers between emacs sessions
;; (require 'psession)
;; (psession-mode 1)

;; save desktop (open files) and load again on start, use only one desktop, save on file open, save more stuff with it
;; (require 'desktop)
;; (setq desktop-restore-eager 10)
;; (setq desktop-save t)
;; (setq desktop-load-locked-desktop t)
;; (setq desktop-save-mode t)

;; (add-to-list 'desktop-modes-not-to-save 'dired-mode)
;; (add-to-list 'desktop-modes-not-to-save 'Info-mode)
;; (add-to-list 'desktop-modes-not-to-save 'Apropos-mode)
;; (add-to-list 'desktop-modes-not-to-save 'info-lookup-mode)
;; (add-to-list 'desktop-modes-not-to-save 'fundamental-mode) ; nope!
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

(setq org-src-fontify-natively t)

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

;; ;; Zoom mode
;; (defun size-callback ()
;;   (cond ((> (frame-pixel-width) 1280) '(90 . 0.6))
;;         (t                            '(0.5 . 0.5))))

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
;; (add-hook 'c-mode-common-hook
;;           (lambda ()
;;             (when (derived-mode-p 'c-mode 'c++-mode 'java-mode)
;;               (ggtags-mode 1))))

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
;; (global-linum-mode t)

;; show (ugly) trailing whitespace
(setq-default show-trailing-whitespace t)

;; Disable trailing whitespace in eshell and term mode
(add-hook 'eshell-mode-hook (lambda () (setq show-trailing-whitespace nil)))
(add-hook 'term-mode-hook (lambda () (setq show-trailing-whitespace nil)))
(add-hook 'shell-mode-hook (lambda () (setq show-trailing-whitespace nil)))

;; Projectile
(projectile-mode +1)
(define-key projectile-mode-map (kbd "s-p") 'projectile-command-map)
(define-key projectile-mode-map (kbd "C-c p") 'projectile-command-map)


(provide 'my_style)
