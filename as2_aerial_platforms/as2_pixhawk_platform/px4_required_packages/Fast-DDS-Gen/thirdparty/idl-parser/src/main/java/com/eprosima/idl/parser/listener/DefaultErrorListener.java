// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

package com.eprosima.idl.parser.listener;

import org.antlr.v4.runtime.BaseErrorListener;
import org.antlr.v4.runtime.RecognitionException;
import org.antlr.v4.runtime.Recognizer;
import com.eprosima.log.ColorMessage;
import com.eprosima.idl.context.Context;
import java.util.Stack;
import com.eprosima.idl.util.Pair;

public class DefaultErrorListener extends BaseErrorListener
{
    public DefaultErrorListener(Context ctx)
    {
        ctx_ = ctx;
    }

    @Override
    public void syntaxError(Recognizer<?, ?> recognizer, Object offendingSymbol,
            int line, int charPositionInLine, String msg, RecognitionException ex)
    {
        // Check if it is a included file.
        Stack<Pair<String, Integer>> stack = ctx_.getScopeFilesStack();

        if(stack.size() > 0)
        {
            boolean first = true;
            for(int count = stack.size() - 1; count >= 0; --count)
            {
                String message = "";

                if(first)
                {
                    message = "In file included from ";
                    first = false;
                }
                else
                {
                    message = "                      ";
                }

                message += ColorMessage.bold(stack.get(count).first());
                message += ":";
                message += ColorMessage.bold(stack.get(count).second().toString());
                message += ":0";

                if(count != 0)
                    message += ",";
                else
                    message += ":";

                System.out.println(message);
            }
        }

        int realLine = line - ctx_.getCurrentIncludeLine();
        System.out.println(ColorMessage.bold(ctx_.getScopeFile() + ":" + realLine + ":" + charPositionInLine + ": ") +
                ColorMessage.red("error: ") + msg);
    }

    private Context ctx_;
}
