;; magit
(global-set-key (kbd "<C-§>") 'magit-status)
(global-set-key [?\C-§] 'magit-status)
(global-set-key [?\C-§] 'magit-dispatch-popup)
(global-set-key (kbd "C-x g") 'magit-status)
(global-set-key (kbd "C-x G") 'magit-dispatch-popup)

;; DWIM prompting when creating new branches.
(setq magit-branch-read-upstream-first 'fallback)

;; Submodule update init recursive
(defun magit-submodule-update-recursive ()
  (interactive)
  (magit-run-git-async "submodule" "sync" "--recursive")
  (magit-run-git-async "submodule" "update" "--init" "--recursive"))

(eval-after-load "magit" '(transient-append-suffix 'magit-submodule "a"
						   '("U" "Update all (recursively)" magit-submodule-update-recursive)))

;; Forge config
;; Forge load after magit. Do not use it so far, not quite useful
;; (with-eval-after-load 'magit
;;   (require 'forge))

;; Github-review config
;; (setq github-review-fetch-top-level-and-review-comments t)

(provide 'my_git)
