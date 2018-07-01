var DEBUG = false;

var IS_IGNITE = (location.pathname.indexOf('/jupyter/') !== -1);
var IGNITE_URL = 'https://www.robotigniteacademy.com';

if (!DEBUG){
    console.log = console.info = function(arg){};
}

var tcKernelID;
function tcRestartKernel(){
    try{
        console.info("tcRestartKernel.begin");
        var url = location.origin + Jupyter.notebook.kernel.kernel_url + '/restart';
        $.post( url, {});
        clearInterval(tcKernelID);
        console.info("tcRestartKernel.end");
    }catch(e){}
}

var TC_IPYTHON = 1;
var TC_IPYTHON_COMMANDS = {
    SAVE: 0
};
var TC_TERMINAL = 2;
var TC_TERMINAL_COMMANDS = {
    CREATE: 0
};
function _processMessage(msg) {
    var obj = JSON.parse(msg);
    switch(obj.type) {
        case TC_IPYTHON:
            _processMessageIPython(obj);
            break;
        case  TC_TERMINAL:
            _processMessageTerminal(obj);
            break;
        default:
            throw "Message type not found!"
    }
}
function _processMessageIPython(obj) {
    switch(obj.cmd) {
        case TC_IPYTHON_COMMANDS.SAVE:
            jQuery("button[data-jupyter-action='jupyter-notebook:save-notebook']").click();
            break;
        default:
            throw "Message command not found!";
    }
}
function _processMessageTerminal(obj) {
    switch(obj.cmd) {
        case TC_TERMINAL_COMMANDS.CREATE:
            __new_terminal();
            break;
        default:
            throw "Message command not found!";
    }
}
function receiveMessage(event) {
    var allowedOrigin = ['http://localhost:8000', 'https://rds.theconstructsim.com', IGNITE_URL];
    console.log(event);
    if(allowedOrigin.indexOf(event.origin) <= -1) {
        console.log("Origin [" + event.origin + "] not allowed");
        return;
    }
    _processMessage(event.data);
}

function __new_terminal() {
    var w = window.open(undefined, '_blank');
    var base_url = '/jupyter_rds/';
    var settings = {
        type : "POST",
        dataType: "json",
        success : function (data, status, xhr) {
            var name = data.name;
            w.location = Jupyter.utils.url_path_join(base_url, 'terminals',  Jupyter.utils.encode_uri_components(name));
        },
        error : function(jqXHR, status, error){
            w.close();
            Jupyter.utils.log_ajax_error(jqXHR, status, error);
        },
    };
    var url = Jupyter.utils.url_path_join(
        base_url,
        'api/terminals'
    );
    $.ajax(url, settings);
}

$(document).ready(function(){
    window.addEventListener('message', receiveMessage);
    $([Jupyter.events]).on("app_initialized.NotebookApp", function () {
        tcKernelID = setInterval(
            function(){
                tcRestartKernel();
                if (IS_IGNITE) {
                    $(".cell").off("dblclick");
                }
            }, 1000
        );

        var custom_css = "/jupyter_rds/custom/rds_notebook.css";
        if (IS_IGNITE){
            $('#header-container, #menubar, #maintoolbar-container div:not(#run_int), #maintoolbar-container select').hide();
            Jupyter.page._resize_site();
            custom_css = "/jupyter/custom/ignite_notebook.css";
            
            // Show command palette
            Jupyter.keyboard_manager.edit_shortcuts.remove_shortcut('cmdtrl-shift-p');
            // Split cell at cursor
            Jupyter.keyboard_manager.edit_shortcuts.remove_shortcut('ctrl-shift--');
            // split cell at cursor
            //Jupyter.keyboard_manager.edit_shortcuts.remove_shortcut('ctrl-shift-subtract');
            
            // Show command palette
            Jupyter.keyboard_manager.command_shortcuts.remove_shortcut('cmdtrl-shift-p');
            // Paste cell above
            Jupyter.keyboard_manager.command_shortcuts.remove_shortcut('shift-v');
            // Merge cells
            Jupyter.keyboard_manager.command_shortcuts.remove_shortcut('shift-m');
            // Enter edit mode
            Jupyter.keyboard_manager.command_shortcuts.remove_shortcut('enter');
            // Interrupt kernel
            Jupyter.keyboard_manager.command_shortcuts.remove_shortcut('i,i');
            // Confirm restart kernel
            Jupyter.keyboard_manager.command_shortcuts.remove_shortcut('0,0');
            // delete cell
            Jupyter.keyboard_manager.command_shortcuts.remove_shortcut('d,d');
            // cut cell
            Jupyter.keyboard_manager.command_shortcuts.remove_shortcut('x');
            // copy cell
            Jupyter.keyboard_manager.command_shortcuts.remove_shortcut('c');
            // paste cell
            Jupyter.keyboard_manager.command_shortcuts.remove_shortcut('v');
            // intert cell above
            Jupyter.keyboard_manager.command_shortcuts.remove_shortcut('a');
            // insert cell below
            Jupyter.keyboard_manager.command_shortcuts.remove_shortcut('b');
            // change cell to code
            Jupyter.keyboard_manager.command_shortcuts.remove_shortcut('y');
            // change cell to markdown
            Jupyter.keyboard_manager.command_shortcuts.remove_shortcut('m');
            // change cell to raw
            Jupyter.keyboard_manager.command_shortcuts.remove_shortcut('r');
            // change cell to heading 1
            Jupyter.keyboard_manager.command_shortcuts.remove_shortcut('1');
            // change cell to heading 2
            Jupyter.keyboard_manager.command_shortcuts.remove_shortcut('2');
            // change cell to heading 3
            Jupyter.keyboard_manager.command_shortcuts.remove_shortcut('3');
            // change cell to heading 4
            Jupyter.keyboard_manager.command_shortcuts.remove_shortcut('4');
            // change cell to heading 5
            Jupyter.keyboard_manager.command_shortcuts.remove_shortcut('5');
            // change cell to heading 6
            Jupyter.keyboard_manager.command_shortcuts.remove_shortcut('6');
            // show keyboard shortcuts
            Jupyter.keyboard_manager.command_shortcuts.remove_shortcut('h');
        }
        $('head').append( $('<link rel="stylesheet" type="text/css" />').attr('href', custom_css) );
    });

    define(['base/js/namespace'], function(Jupyter) {
        Jupyter._target = '_self';
    });
    $(document).click(function() {
        try {
            if(DEBUG) {
                parent.postMessage(window.location.href, 'http://localhost:8000');
            } else if(IS_IGNITE) {
                parent.postMessage(window.location.href, IGNITE_URL);
            }
        } catch(e) {
            console.log('it\'s ok');
        }
    });


    if (location.pathname.match('terminals')) {
        var check_sh_used = location.search.split('check_sh_used=')[1];
        if (check_sh_used == "true") {
            setTimeout(function(){
                terminal.term.on('data', function(e){
                    parent.postMessage(JSON.stringify({type : 'shell', data : 'used'}), '*');
                });
            }, 5000);
        }
    }


    if (IS_IGNITE) {

        //////////////////////////////////////////////////////////////////////
        // Keeping the notebook on the same position when (maxi|mini)mizing
        //////////////////////////////////////////////////////////////////////

        var top_element = null;
        var top_element_position = null;
        var diff = null;

        $("#site").scroll(function () {
            setTimeout(function () {
                top_element = document.elementFromPoint($(this).width() / 2, 200);
                top_element_position = top_element.getBoundingClientRect().y;
            }, 300);
        });
        $(window).resize(function () {
            if (top_element !== null) {
                diff = top_element.getBoundingClientRect().y - top_element_position;
                $("#site").scrollTop($("#site").scrollTop() + diff);
            }
        });
    }
});

