; start package.el with emacs
(require 'package)

;; add MELPA to repository list
(add-to-list 'package-archives '("melpa" . "https://melpa.org/packages/") t)
;; Comment/uncomment this line to enable MELPA Stable if desired.  See `package-archive-priorities`
;; and `package-pinned-packages`. Most users will not need or want to do this.
;; (add-to-list 'package-archives '("melpa-stable" . "https://stable.melpa.org/packages/") t)
;; initialize package.el
(package-initialize)

;; add local lisp folder to load-path
(add-to-list 'load-path "~/.emacs.d/lisp")

;; add local custom configs to load path
(add-to-list 'load-path "~/.emacs.d/configs")


;; set default directory (e.g. for C-x-s and C-x-f)
(setq default-directory "~/")


(require 'my_style)
(require 'my_leetcode)
(require 'my_git)
(require 'my_keybindings)
(require 'my_autocompletion)
(require 'my_multiterm)
(require 'my_projectile)
(require 'my_helm)
(require 'my_dap_mode)
(require 'my_org_mode)
(require 'my_pdf_mode)
(require 'my_clang_format)

(provide 'init)
(custom-set-variables
 ;; custom-set-variables was added by Custom.
 ;; If you edit it by hand, you could mess it up, so be careful.
 ;; Your init file should contain only one such instance.
 ;; If there is more than one, they won't work right.
 '(custom-enabled-themes '(modus-vivendi use-package))
 '(package-selected-packages
   '(doom-modeline modus-themes zenburn-theme yaml-mode tramp-term tramp spacemacs-theme solo-jazz-theme solarized-theme quelpa-use-package pdf-tools ox-latex-subfigure org-noter multiple-cursors multi-term material-theme magit leuven-theme highlight-parentheses helm-w3m helm-tramp helm-projectile helm-gtags helm-bibtex helm-ag ggtags ein dockerfile-mode docker-tramp dap-mode cuda-mode copilot company-tabnine company-irony-c-headers company-irony company-anaconda color-theme-sanityinc-tomorrow cmake-mode clang-format+ autopair auctex apropospriate-theme anti-zenburn-theme ag)))
(custom-set-faces
 ;; custom-set-faces was added by Custom.
 ;; If you edit it by hand, you could mess it up, so be careful.
 ;; Your init file should contain only one such instance.
 ;; If there is more than one, they won't work right.
 )
