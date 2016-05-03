if &cp | set nocp | endif
let s:cpo_save=&cpo
set cpo&vim
nmap gx <Plug>NetrwBrowseX
nnoremap <silent> <Plug>NetrwBrowseX :call netrw#NetrwBrowseX(expand("<cWORD>"),0)
cabbr tabrosed =(getcmdtype()==':' && getcmdpos()==1 ? 'TabRosed' : 'tabrosed')
cabbr rosed =(getcmdtype()==':' && getcmdpos()==1 ? 'Rosed' : 'rosed')
cabbr roscd =(getcmdtype()==':' && getcmdpos()==1 ? 'Roscd' : 'roscd')
let &cpo=s:cpo_save
unlet s:cpo_save
set backspace=indent,eol,start
set fileencodings=ucs-bom,utf-8,default,latin1
set helplang=en
set history=50
set makeprg=catkin_make\ -C\ ~/catkin_ws\ \ --pkg\ train
set nomodeline
set printoptions=paper:a4
set ruler
set runtimepath=~/.vim,~/.vim/plugged/vim-ros/,/var/lib/vim/addons,/usr/share/vim/vimfiles,/usr/share/vim/vim74,/usr/share/vim/vimfiles/after,/var/lib/vim/addons/after,~/.vim/after
set suffixes=.bak,~,.swp,.o,.info,.aux,.log,.dvi,.bbl,.blg,.brf,.cb,.ind,.idx,.ilg,.inx,.out,.toc
" vim: set ft=vim :
