;; line wrap, line numbering
(global-set-key (kbd "<f10>") 'toggle-truncate-lines)
;; (global-set-key (kbd "<S-f10>") 'linum-mode)

;; delete word forward and backwards
(global-set-key (kbd "<M-delete>") 'delete-word)
(global-set-key (kbd "<M-backspace>") 'backward-delete-word)

;; hippie-expand instead of dabrev
(global-set-key (kbd "M-/") 'hippie-expand)
(global-set-key (kbd "<C-tab>") 'hippie-expand)

;; better buffer menu
(global-set-key (kbd "C-x C-b") 'ibuffer)

;; swap regex/non-regex interactive search
(global-set-key (kbd "C-s") 'isearch-forward-regexp)
(global-set-key (kbd "C-r") 'isearch-backward-regexp)
(global-set-key (kbd "C-M-s") 'isearch-forward)
(global-set-key (kbd "C-M-r") 'isearch-backward)

;; switch header/source
(global-set-key (kbd "C-x C-o") 'projectile-find-other-file)

;; ;; jump between matching parenthesis
;; ;; (global-set-key "%" 'match-paren)
;; (defvar my-cpp-other-file-alist
;;   '(("\\.cpp\\'" (".h" ".hpp" ".ipp"))
;;     ("\\.ipp\\'" (".hpp" ".cpp"))
;;     ("\\.hpp\\'" (".ipp" ".cpp"))
;;     ("\\.cxx\\'" (".hxx" ".ixx"))
;;     ("\\.ixx\\'" (".cxx" ".hxx"))
;;     ("\\.hxx\\'" (".ixx" ".cxx"))
;;     ("\\.c\\'" (".h"))
;;     ("\\.h\\'" (".cpp" ".c"))
;;     ))

;; (setq-default ff-other-file-alist 'my-cpp-other-file-alist)

;; (setq ff-search-directories
;;       '("." "../src" "../include" "../../src" "../../include" ))

;; C-o switches to other window (if more than one window)
(global-set-key (kbd "C-o") 'other-window)

;; M-g to jump to line number
(global-set-key (kbd "M-g") 'goto-line)

;; M-up / M-down to scroll while keeping the cursor (as far as possible)
(global-set-key (kbd "<M-down>") 'scroll-down-keep-cursor) ; [M-down]
(global-set-key (kbd "<M-up>") 'scroll-up-keep-cursor)


;; Multiple cursors
(require 'multiple-cursors)
(global-set-key (kbd "C-S-c C-S-c") 'mc/edit-lines)

(global-set-key (kbd "C->") 'mc/mark-next-like-this)
(global-set-key (kbd "C-<") 'mc/mark-previous-like-this)
(global-set-key (kbd "C-c C-<") 'mc/mark-all-like-this)

;; helm gtags
(global-set-key (kbd "M-.") 'helm-gtags-dwim)
(global-set-key (kbd "M-,") 'helm-gtags-pop-stack)

(provide 'my_keybindings)
