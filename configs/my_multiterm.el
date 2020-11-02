;;; Commentary:
;;; this is the multi-term configuration based on
;;; http://rawsyntax.com/blog/learn-emacs-zsh-and-multi-term/
;;; and
;;; https://www.emacswiki.org/emacs/MultiTerm
;;; Code:

(require 'multi-term)

(setq multi-term-program "/bin/zsh")

(add-hook 'term-mode-hook
          (lambda ()
            (setq term-buffer-maximum-size 10000)))

(setq multi-term-scroll-show-maximum-output t)

(defun term-toggle-line-char-mode ()
  (interactive)
  (if (term-in-line-mode)
      (term-char-mode)
    (term-line-mode)))

(add-hook 'term-mode-hook
          (lambda ()
            (setq show-trailing-whitespace nil)))

(setq term-unbind-key-list
  '("C-z" "C-x" "C-c" "C-h" "C-y" "<ESC>"))

(setq term-bind-key-alist
  '(
    ("C-c C-c" . term-interrupt-subjob)
    ("M-p" . previous-line)
    ("M-n" . next-line)
    ("C-s" . isearch-forward)
    ("C-r" . isearch-backward)
    ("C-m" . term-send-raw)
    ("M-f" . term-send-forward-word)
    ("M-b" . term-send-backward-word)
    ("M-o" . term-send-backspace)
    ("C-p" . term-send-up)
    ("C-n" . term-send-down)
    ("M-d" . term-send-forward-kill-word)
    ("M-<backspace>" . term-send-backward-kill-word)
    ("M-r" . term-send-reverse-search-history)
    ("M-," . term-send-input)
    ("M-." . comint-dynamic-complete)))


(add-hook 'term-mode-hook
          (lambda ()
            (add-to-list 'term-bind-key-alist '("M-[" . multi-term-prev))
            (add-to-list 'term-bind-key-alist '("M-]" . multi-term-next))
	    (add-to-list 'term-bind-key-alist '("C-c C-j" . term-toggle-line-char-mode)) ;char-mode
	    (define-key term-mode-map (kbd "C-c C-j")      'term-toggle-line-char-mode)  ;line-mode
	    (define-key term-raw-map (kbd "C-y") 'term-paste)
	    )
	  )

;; different cursor for line and char mode
(defun multi-term-set-cursor-according-to-mode ()
  "Change cursor type according to multi-term mode."
  (cond
   ((term-in-char-mode)
    (setq cursor-type '(hbar . 3)))
   (t
    (setq cursor-type '(bar . 3)))))

(eval-after-load "multi-term"
  '(progn
     (add-hook 'post-command-hook 'multi-term-set-cursor-according-to-mode)))

(provide 'my_multiterm)
