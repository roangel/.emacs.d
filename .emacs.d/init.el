;; start package.el with emacs
(require 'package)

;; add MELPA to repository list
(add-to-list 'package-archives
             '("melpa-stable" . "https://stable.melpa.org/packages/") t)
;; initialize package.el
(package-initialize)

;; add local lisp folder to load-path
(add-to-list 'load-path "~/.emacs.d/lisp")

;; add local custom configs to load path
(add-to-list 'load-path "~/.emacs.d/configs")


;; set default directory (e.g. for C-x-s and C-x-f)
(setq default-directory "~/")

(setq custom-file (concat user-emacs-directory "/configs/my_custom.el"))
(load-file custom-file)

(require 'my_style)
(require 'my_leetcode)
(require 'my_git)
(require 'my_keybindings)
(require 'my_autocompletion)
(require 'my_multiterm)
(require 'my_projectile)
(require 'my_helm)

(provide 'init)
